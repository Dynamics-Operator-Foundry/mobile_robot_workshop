import numpy as np
from math import pi
from common.lx16a import *
import time


class Motor:
    """Motor controller for multiple servos with velocity measurement."""

    def __init__(self, servo_ids=[2, 3], port="/dev/ttyUSB0", window_size=10):
        """
        Initialize motor controller.

        Args:
            servo_ids: List of servo IDs
            port: Serial port
            window_size: Number of samples for velocity calculation
        """
        # Initialize communication
        LX16A.initialize(port, 0.1)

        # Create servos
        self.servo_ids = servo_ids
        self.num_servos = len(servo_ids)
        self.servos = [LX16A(sid) for sid in servo_ids]

        # Stop all motors initially
        for servo in self.servos:
            servo.motor_mode(0)

        # Velocity tracking
        self.window_size = window_size
        self.timestamps = np.zeros(window_size)
        self.unwrapped_angles = np.zeros((window_size, self.num_servos))
        self.raw_angles = np.zeros(self.num_servos)
        self.prev_angles = np.zeros(self.num_servos)
        self.motor_vel = np.zeros(self.num_servos)  # Current velocities in rad/s
        self.target_speeds = np.zeros(self.num_servos, dtype=int)

        self.sample_idx = 0
        self.is_initialized = False

    def start(self, speeds):
        """
        Start motors at given speeds.

        Args:
            speeds: Array-like of motor speeds (-1000 to 1000)
        """
        speeds = np.array(speeds, dtype=int)
        self.target_speeds = speeds
        for i, servo in enumerate(self.servos):
            servo.motor_mode(int(speeds[i]))

    def stop(self):
        """Stop all motors."""
        self.target_speeds = np.zeros(self.num_servos, dtype=int)
        for servo in self.servos:
            servo.motor_mode(0)

    def update(self):
        """Update velocity measurements. Call this regularly (e.g., every 50ms)."""
        # Read all servo angles
        for i, servo in enumerate(self.servos):
            self.raw_angles[i] = servo.get_physical_angle()

        # Unwrap angles (handle 360° wraparound)
        if self.sample_idx == 0 and not self.is_initialized:
            # First sample - initialize
            unwrapped = self.raw_angles.copy()
        else:
            # Calculate delta and correct for wraparound
            delta = self.raw_angles - self.prev_angles
            delta[delta < -180] += 360
            delta[delta > 180] -= 360

            prev_idx = (self.sample_idx - 1) % self.window_size
            unwrapped = self.unwrapped_angles[prev_idx] + delta

        # Store data in circular buffer
        self.timestamps[self.sample_idx] = time.time()
        self.unwrapped_angles[self.sample_idx] = unwrapped
        self.prev_angles = self.raw_angles.copy()

        # Move to next index
        self.sample_idx = (self.sample_idx + 1) % self.window_size

        # Mark as initialized after first full window
        if not self.is_initialized and self.sample_idx == 0:
            self.is_initialized = True

        # Calculate velocities if we have enough samples
        if self.is_initialized:
            # Use oldest and newest samples
            oldest_idx = self.sample_idx
            newest_idx = (self.sample_idx - 1) % self.window_size

            dt = self.timestamps[newest_idx] - self.timestamps[oldest_idx]
            if dt > 0:
                # Velocity in rad/s
                self.motor_vel = (self.unwrapped_angles[newest_idx] - self.unwrapped_angles[oldest_idx]) * (pi / 180.0) / dt

    def get_vel(self):
        """
        Get current velocities.

        Returns:
            Array of velocities in rad/s
        """
        return self.motor_vel.copy()

# Example usage
if __name__ == "__main__":
    # Create motor controller
    motor = Motor(servo_ids=[2, 3], window_size=10)

    # Start motors
    # motor.start([1000, 500]) # 1000 --> 4.8 rad/s at zero torque

    print(f"\n{'Time (s)':<10} {'Speed1':<10} {'Speed2':<10} {'ω1 (rad/s)':<13} {'ω2 (rad/s)':<13}")
    print("-" * 66)

    start_time = time.time()

    try:
        while True:
            t = time.time() - start_time

            # Update motor measurements
            motor.update()
            # set a sine wave 
            motor.start(np.clip([500+np.sin(t)*1000, -np.sin(t)*1000], -1000, 1000))


            # Get velocities
            vel = motor.get_vel()

            # Print
            print(f"{t:<10.2f} {motor.target_speeds[0]:<10} {motor.target_speeds[1]:<10} {vel[0]:<13.4f} {vel[1]:<13.4f}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        motor.stop()
        print("\n\nMotors stopped.")
