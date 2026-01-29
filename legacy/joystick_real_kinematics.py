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

  # --- ADDED (optional) ---
  # WHEEL_TRACK_M, WHEEL_DIAM_M, MAX_VX, MAX_W, ENABLE_GEOM_MAP
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

WHEEL_TRACK_M    = _env_float("WHEEL_TRACK_M", 0.30)
WHEEL_DIAM_M     = _env_float("WHEEL_DIAM_M", 0.12)
_WHEEL_RADIUS_M  = WHEEL_DIAM_M * 0.5
MAX_VX           = _env_float("MAX_VX", 1.0)
MAX_W            = _env_float("MAX_W", 2.5)
ENABLE_GEOM_MAP  = _env_int("ENABLE_GEOM_MAP", 1) >= 1

def compute_wheel_rps_geom(v_fwd_cmd: float, v_turn_cmd: float) -> Tuple[float, float]:

    #Map normalized forward/turn commands in [-1,1] to *mechanical* wheel RPS
    #using differential-drive kinematics and real geometry.

    v_fwd_cmd ∈ [-1,1] -> vx ∈ [-MAX_VX, MAX_VX]      (m/s)
    v_turn_cmd ∈ [-1,1] -> w  ∈ [-MAX_W,  MAX_W ]     (rad/s)

    wR = (vx + 0.5 * C * w) / r
    wL = (vx - 0.5 * C * w) / r

    vx = float(np.clip(v_fwd_cmd, -1.0, 1.0)) * MAX_VX
    w  = float(np.clip(v_turn_cmd, -1.0, 1.0)) * MAX_W

    C  = WHEEL_TRACK_M
    r  = max(_WHEEL_RADIUS_M, 1e-9)

    wR_rad = (vx + 0.5 * C * w) / r
    wL_rad = (vx - 0.5 * C * w) / r

    wR_rps = wR_rad / (2.0 * np.pi)
    wL_rps = wL_rad / (2.0 * np.pi)
    return float(wL_rps), float(wR_rps)
# --- END ADDED --------------------------------------------------------------


def normalize_axis(raw: int) -> float:

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
    axis_turn = os.getenv("AXIS_TURN", "ABS_RX")
    invert_fwd = 1 if _env_int("INVERT_FWD", 1) >= 0 else -1
    invert_turn = 1 if _env_int("INVERT_TURN", 1) >= 0 else -1

    print(f"Listening for joystick UDP on {ip}:{port}")
    print(f"Using axes: forward={axis_fwd} turn={axis_turn} deadzone={deadzone}")
    print(f"Safety timeout: {cmd_timeout_s}s  max_wheel_rps={max_wheel_rps}")

    # Start joystick receiver
    rx = UdpJoystickReceiver(ip, port)
    rx.start()

    # Setup motor controller
    motor = CanMotorController(motor_setup)
    motor.set_max_torque_ratio(0.2)
    motor.start_motion_control_continuously()
