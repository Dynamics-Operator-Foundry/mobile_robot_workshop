import time
import numpy as np
from math import pi, atan2, hypot, sin, cos
from common.lx16a import LX16A
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

class Motor:
    def __init__(self, servo_ids=[2, 3], port="/dev/ttyUSB0", window_size=10):
        LX16A.initialize(port, 0.1)
        self.servo_ids = servo_ids
        self.num_servos = len(servo_ids)
        self.servos = [LX16A(sid) for sid in servo_ids]
        for servo in self.servos:
            servo.motor_mode(0)
        self.window_size = window_size
        self.timestamps = np.zeros(window_size)
        self.unwrapped_angles = np.zeros((window_size, self.num_servos))
        self.raw_angles = np.zeros(self.num_servos)
        self.prev_angles = np.zeros(self.num_servos)
        self.motor_vel = np.zeros(self.num_servos)
        self.target_speeds = np.zeros(self.num_servos, dtype=int)
        self.sample_idx = 0
        self.is_initialized = False

    def start(self, speeds):
        speeds = np.array(speeds, dtype=int)
        self.target_speeds = speeds
        for i, servo in enumerate(self.servos):
            servo.motor_mode(int(speeds[i]))

    def stop(self):
        self.target_speeds = np.zeros(self.num_servos, dtype=int)
        for servo in self.servos:
            servo.motor_mode(0)

    def update(self):
        for i, servo in enumerate(self.servos):
            self.raw_angles[i] = servo.get_physical_angle()
        if self.sample_idx == 0 and not self.is_initialized:
            unwrapped = self.raw_angles.copy()
        else:
            delta = self.raw_angles - self.prev_angles
            delta[delta < -180] += 360
            delta[delta >  180] -= 360
            prev_idx = (self.sample_idx - 1) % self.window_size
            unwrapped = self.unwrapped_angles[prev_idx] + delta
        self.timestamps[self.sample_idx] = time.time()
        self.unwrapped_angles[self.sample_idx] = unwrapped
        self.prev_angles = self.raw_angles.copy()
        self.sample_idx = (self.sample_idx + 1) % self.window_size
        if not self.is_initialized and self.sample_idx == 0:
            self.is_initialized = True
        if self.is_initialized:
            oldest_idx = self.sample_idx
            newest_idx = (self.sample_idx - 1) % self.window_size
            dt = self.timestamps[newest_idx] - self.timestamps[oldest_idx]
            if dt > 0:
                self.motor_vel = (self.unwrapped_angles[newest_idx] - self.unwrapped_angles[oldest_idx]) * (pi / 180.0) / dt

    def get_vel(self):
        return self.motor_vel.copy()

track_width = 0.136
wheel_diameter = 0.077
wheel_radius = wheel_diameter * 0.5
half_track_width = track_width * 0.5
max_angular_speed = 4.8

RAD_PER_CMD = 4.8 / 1000.0
CMD_PER_RAD = 1.0 / RAD_PER_CMD
CMD_MIN, CMD_MAX = -1000, 1000

LEFT_CMD_SIGN  = 1
RIGHT_CMD_SIGN = -1

V_MAX = 0.4
W_MAX = 0.7
TH_DEADBAND = 3.0 * pi / 180.0

def normalize_angle(a):
    x = (a + pi) % (2 * pi) - pi
    return x if x != -pi else pi

def plan_from_keyboard():
    raw_line = input().strip()
    dx, dy, dw = map(float, raw_line.split())
    return dx, dy, dw

def wheel_radps_to_cmd(omega):
    return int(np.clip(omega * CMD_PER_RAD, CMD_MIN, CMD_MAX))

def yaw_from_quat(qx, qy, qz, qw):
    return atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

class OdomPose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.stamp = 0.0
        self.have_pose = False
        self.lock = threading.Lock()

    def set_pose(self, x, y, th, stamp):
        with self.lock:
            self.x, self.y, self.th, self.stamp, self.have_pose = x, y, normalize_angle(th), stamp, True

    def get(self):
        with self.lock:
            return self.x, self.y, self.th, self.stamp, self.have_pose

class OdomListener(Node):
    def __init__(self, pose_store: OdomPose):
        super().__init__('odom_listener')
        self.pose_store = pose_store
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Odometry, '/camera/odom/sample', self.odom_cb, qos)
        self.create_subscription(Odometry, '/camera/pose/sample', self.odom_cb, qos)

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        th = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.pose_store.set_pose(x, y, th, msg.header.stamp.sec + 1e-9*msg.header.stamp.nanosec)

RELATIVE_INPUT = True

def compute_wheel_radps_from_vw(v, w):
    v_l = v - w * half_track_width
    v_r = v + w * half_track_width
    om_l = np.clip(v_l / wheel_radius, -max_angular_speed, max_angular_speed)
    om_r = np.clip(v_r / wheel_radius, -max_angular_speed, max_angular_speed)
    return om_l, om_r

def send_wheels(motor: Motor, om_l, om_r, swap_lr=False, invert=False):
    lc = wheel_radps_to_cmd(om_l)
    rc = wheel_radps_to_cmd(om_r)
    if invert:
        lc, rc = -lc, -rc
    lc *= LEFT_CMD_SIGN
    rc *= RIGHT_CMD_SIGN
    if swap_lr:
        lc, rc = rc, lc
    motor.start([lc, rc])

def p_go_to(motor: Motor,
            pose: OdomPose,
            x_d, y_d, th_d,
            Kp_lin=0.6, Kp_ang=1.2,
            dt=0.05,
            stop_r=0.02, stop_th=4.0*pi/180.0,
            swap_lr=False, invert=False,
            max_time=15.0):

    t0 = time.time()

    while True:
        rclpy.spin_once(p_go_to.node, timeout_sec=0.0)
        x, y, th, _, have = pose.get()
        if not have:
            time.sleep(0.02); continue
        ex, ey = x_d - x, y_d - y
        if hypot(ex, ey) < 1e-3: break
        theta_path = atan2(ey, ex)
        err = normalize_angle(theta_path - th)
        if abs(err) <= stop_th: break
        v = 0.0
        w = float(np.clip(Kp_ang * err, -W_MAX, W_MAX))
        if abs(w) < TH_DEADBAND: w = 0.0
        om_l, om_r = compute_wheel_radps_from_vw(v, w)
        motor.update(); send_wheels(motor, om_l, om_r, swap_lr, invert)
        if time.time() - t0 > max_time: motor.stop(); return False
        time.sleep(dt)

    while True:
        rclpy.spin_once(p_go_to.node, timeout_sec=0.0)
        x, y, th, _, have = pose.get()
        if not have:
            time.sleep(0.02); continue
        ex, ey = x_d - x, y_d - y
        epos = hypot(ex, ey)
        if epos <= stop_r:
            motor.stop(); break
        theta_path = atan2(ey, ex)
        heading_err = normalize_angle(theta_path - th)
        v = float(np.clip(Kp_lin * epos, -V_MAX, V_MAX))
        w = float(np.clip(Kp_ang * heading_err, -W_MAX, W_MAX))
        if abs(w) < TH_DEADBAND: w = 0.0
        om_l, om_r = compute_wheel_radps_from_vw(v, w)
        motor.update(); send_wheels(motor, om_l, om_r, swap_lr, invert)
        if time.time() - t0 > max_time: motor.stop(); return False
        time.sleep(dt)
        
    while True:
        rclpy.spin_once(p_go_to.node, timeout_sec=0.0)
        x, y, th, _, have = pose.get()
        if not have:
            time.sleep(0.02); continue
        eth = normalize_angle(th_d - th)
        if abs(eth) <= stop_th:
            motor.stop(); return True
        v = 0.0
        w = float(np.clip(Kp_ang * eth, -W_MAX, W_MAX))
        if abs(w) < TH_DEADBAND: w = 0.0
        om_l, om_r = compute_wheel_radps_from_vw(v, w)
        motor.update(); send_wheels(motor, om_l, om_r, swap_lr, invert)
        if time.time() - t0 > max_time: motor.stop(); return False
        time.sleep(dt)

def main():
    rclpy.init(args=None)
    pose = OdomPose()
    node = OdomListener(pose)
    p_go_to.node = node
    motor = Motor(servo_ids=[2, 3], window_size=10)
    try:
        # wait for first pose
        t0 = time.time()
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            _, _, _, _, have = pose.get()
            if have: break
            if time.time() - t0 > 2.0: t0 = time.time()

        while True:
            dx, dy, dth = plan_from_keyboard()
            x, y, th, _, _ = pose.get()
            if RELATIVE_INPUT:
                x_d = x + dx * cos(th) - dy * sin(th)
                y_d = y + dx * sin(th) + dy * cos(th)
                th_d = normalize_angle(th + dth)
            else:
                x_d, y_d, th_d = dx, dy, dth

            p_go_to(motor, pose,
                    x_d, y_d, th_d,
                    Kp_lin=0.6, Kp_ang=1.2,
                    dt=0.05,
                    stop_r=0.02, stop_th=4.0*pi/180.0,
                    swap_lr=False, invert=False,
                    max_time=20.0)
    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
