"""
Joystick-to-motor bridge for the rover using UDP input from scripts/tx.py.

Setup (Linux CAN and environment):
  sudo ip link set can0 down
  sudo ip link set can0 up type can bitrate 1000000
  sudo ifconfig can0 txqueuelen 128

Run:
  micromamba activate py312
  python joystick_rover_control.py

Environment variables (optional):
  JOY_UDP_IP             default: 0.0.0.0 (listen on all)
  JOY_UDP_PORT           default: 5005
  DEADZONE               default: 0.06  (0..1)
  CMD_TIMEOUT_S          default: 0.3   (s without packets -> stop)
  MAX_WHEEL_RPS          default: 6.0   (target wheel mech rps)
  AXIS_FWD               default: ABS_Y (forward/back)
  AXIS_TURN              default: ABS_RX (left/right yaw)
  INVERT_FWD             default: 1     (1 normal, -1 invert)
  INVERT_TURN            default: 1     (1 normal, -1 invert)
  NONLINEAR_POWER        default: 2.0   (power for non-linear mapping, 2.0=quadratic)
  PERSISTENT_ROTATION    default: 1     (1=enabled, 0=disabled - keeps rotation at joystick limits)
"""

import json
import os
import signal
import socket
import sys
import threading
import time
from typing import Dict, Optional, Tuple

import numpy as np                      # math utilities for normalization and mixing

# Ensure this script and the DukeHumanoidV2/control dir are importable from repo root
repo_root = os.path.dirname(__file__)

from DukeHumanoidV2.control.motor.py_motor import CanMotorController  # motor driver over CAN
from DukeHumanoidV2.control.modular_drone_motor_config import motor_setup  # motor CAN IDs and bus config


def _env_float(name: str, default: float) -> float:
    # Read a float from environment or fall back to default
    try:
        return float(os.getenv(name, str(default)))
    except Exception:
        return default


def _env_int(name: str, default: int) -> int:
    # Read an int from environment or fall back to default
    try:
        return int(os.getenv(name, str(default)))
    except Exception:
        return default


def normalize_axis(raw: int) -> float:
    """Normalize various common joystick ranges to [-1, 1] from 16-bit signed (-32768..32767)
    """
    # Heuristics based on magnitude
    if raw <= -2000 or raw >= 2000:
        # Assume signed 16-bit
        # Center ~0, span ~32767
        v = np.clip(raw / 32767.0, -1.0, 1.0)
        return float(v)

    # Check narrower unsigned ranges before wider ones
    if 0 <= raw <= 255:
        center = 127.5
        span = 127.5
        v = (raw - center) / span
        return float(np.clip(v, -1.0, 1.0))

    if 0 <= raw <= 1023:
        center = 511.5
        span = 511.5
        v = (raw - center) / span
        return float(np.clip(v, -1.0, 1.0))

    # Fallback: treat as signed 16-bit
    v = np.clip(raw / 32767.0, -1.0, 1.0)
    return float(v)


# Receives joystick axis JSON over UDP in a background thread and exposes latest state
class UdpJoystickReceiver:
    # Background UDP listener for JSON axis packets
    def __init__(self, ip: str, port: int) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((ip, port))
        self._latest_axes: Dict[str, int] = {}   # last received axis dictionary
        self._last_rx_time: float = 0.0          # timestamp of last valid packet
        self._lock = threading.Lock()            # protects shared state
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        # Start background receive thread
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        # Stop background thread and close socket
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=0.5)

    def _rx_loop(self) -> None:
        # Blocking receive loop: updates latest axes and timestamp
        while self._running:
            try:
                data, _ = self._sock.recvfrom(2048)
                msg = json.loads(data)
                if isinstance(msg, dict):
                    with self._lock:
                        # Expect dict like {"ABS_X": val, ...}
                        self._latest_axes = {str(k): int(v) for k, v in msg.items()}
                        self._last_rx_time = time.time()
            except OSError:
                break
            except Exception:
                # Ignore malformed packets
                continue

    def get_state(self) -> Tuple[Dict[str, int], float]:
        # Snapshot latest axes and last receive time
        with self._lock:
            return dict(self._latest_axes), self._last_rx_time


def apply_deadzone(value: float, deadzone: float) -> float:
    # Apply deadzone and re-scale to keep full stick travel usable
    if abs(value) < deadzone:
        return 0.0
    # Re-scale so that output ramps from 0 at the edge of the deadzone
    sign = 1.0 if value >= 0 else -1.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return sign * np.clip(scaled, 0.0, 1.0)


def apply_nonlinear_mapping(value: float, power: float = 2.0) -> float:
    """
    Apply non-linear mapping to joystick input for more responsive control.
    
    Args:
        value: Input value in range [-1, 1]
        power: Power for non-linear mapping (2.0 = quadratic, higher = more aggressive)
    
    Returns:
        Non-linearly mapped value in range [-1, 1]
    """
    if abs(value) < 1e-6:  # Avoid division by zero
        return 0.0
    
    sign = 1.0 if value >= 0 else -1.0
    abs_value = abs(value)
    
    # Apply power function for non-linear response
    mapped = sign * (abs_value ** power)
    
    return float(np.clip(mapped, -1.0, 1.0))


def compute_wheel_rps(v_fwd_cmd: float, v_turn_cmd: float, max_rps: float) -> Tuple[float, float]:
    # Differential drive mixing: forward +/- turn scaled by max_rps
    w_left = (v_fwd_cmd - v_turn_cmd) * max_rps
    w_right = (v_fwd_cmd + v_turn_cmd) * max_rps
    return float(w_left), float(w_right)


def main() -> int:
    # Configuration (override via environment variables)
    ip = os.getenv("JOY_UDP_IP", "0.0.0.0")
    port = _env_int("JOY_UDP_PORT", 5005)
    # Clamp deadzone to [0.0, 0.95] to avoid degenerate scaling
    deadzone = float(np.clip(_env_float("DEADZONE", 0.06), 0.0, 0.95))
    cmd_timeout_s = _env_float("CMD_TIMEOUT_S", 0.3)
    # Ensure non-negative maximum wheel speed
    max_wheel_rps = max(0.0, _env_float("MAX_WHEEL_RPS", 6.0))
    axis_fwd = os.getenv("AXIS_FWD", "ABS_Y")
    axis_turn = os.getenv("AXIS_TURN", "ABS_X")
    invert_fwd = 1 if _env_int("INVERT_FWD", 1) >= 0 else -1
    invert_turn = 1 if _env_int("INVERT_TURN", 1) >= 0 else -1
    nonlinear_power = max(0.1, _env_float("NONLINEAR_POWER", 2.0))  # Ensure positive power
    persistent_rotation = _env_int("PERSISTENT_ROTATION", 1) > 0

    print(f"Listening for joystick UDP on {ip}:{port}")
    print(f"Using axes: forward={axis_fwd} turn={axis_turn} deadzone={deadzone}")
    print(f"Safety timeout: {cmd_timeout_s}s  max_wheel_rps={max_wheel_rps}")
    print(f"Non-linear power: {nonlinear_power}  persistent_rotation: {persistent_rotation}")

    # Start joystick receiver
    rx = UdpJoystickReceiver(ip, port)
    rx.start()

    # Setup motor controller
    motor = CanMotorController(motor_setup)
    motor.set_max_torque_ratio(0.2)
    motor.start_motion_control_continuously()
    motor.loc_kp[:] = 0
    motor.spd_kp[:] = 2
    motor.enable()
    motor.set_pos_zero()

    # Graceful shutdown handling (Ctrl+C, SIGTERM)
    shutdown = {"flag": False}

    def _handle_sig(signum, frame):
        shutdown["flag"] = True

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    raw_fwd = 0

    try:
        dt = 0.01
        # Variables to track persistent rotation state
        last_valid_fwd = 0.0
        last_valid_turn = 0.0
        last_valid_time = 0.0
        
        while not shutdown["flag"]:
            axes, last_rx = rx.get_state()  # latest joystick state
            now = time.time()

            # Get normalized commands from selected axes
            raw_fwd = axes.get(axis_fwd, 0)
            raw_turn = axes.get(axis_turn, 0)
        
            
            if (raw_fwd - 1024)/2048 * 2 + (raw_turn - 1024)/2048 * 2 < -1.99999999999:
                # avoid at start -1 values
                continue
            
            fwd = (raw_fwd - 1024)/2048 * 2
            turn = -(raw_turn - 1024)/2048 * 2

            # Handle timeout and persistent rotation
            if now - last_rx > cmd_timeout_s:
                if persistent_rotation:
                    # Keep using last valid commands if persistent rotation is enabled
                    # Only stop if we've been without input for a very long time (5x timeout)
                    if now - last_valid_time > cmd_timeout_s * 5.0:
                        fwd = 0.0
                        turn = 0.0
                    else:
                        fwd = last_valid_fwd
                        turn = last_valid_turn
                else:
                    # Stop immediately on timeout
                    fwd = 0.0
                    turn = 0.0
            else:
                # Update last valid commands when we have fresh input
                last_valid_fwd = fwd
                last_valid_turn = turn
                last_valid_time = now

            w_left, w_right = compute_wheel_rps(fwd, turn, max_wheel_rps)

            motor.mech_vel_ref[0] = w_right  # left wheel target mech rps
            motor.mech_vel_ref[1] = -w_left  # right wheel target mech rps (right hand rule)

            time.sleep(dt)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Stop motors safely
        try:
            motor.mech_vel_ref[0] = 0.0
            motor.mech_vel_ref[1] = 0.0
            time.sleep(0.05)
        except Exception:
            pass
        try:
            motor.disable(False)
        except Exception:
            pass
        rx.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())

