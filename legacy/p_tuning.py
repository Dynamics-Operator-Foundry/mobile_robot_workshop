import time
import numpy as np
from math import pi, atan2, hypot, sin, cos
from common.lx16a import LX16A
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry




wheel_diameter = 0.077
wheel_radius   = wheel_diameter * 0.5
track_width    = 0.136

Kp_x_fwd           = 0.9
Kp_y_lat           = 0.3
Kp_psi_yaw         = 2.2
Kp_distance        = 0.8
Kp_heading_bearing = 2.0
Kp_psi_final       = 2.0
D_blend            = 0.35

v_max        = 0.6
w_max        = 3.0
wheel_w_max  = 40.0

position_deadband = 0.02
yaw_deadband      = 0.035

sample_time = 0.02  # 50 Hz
ALPHA_yaw   = 0.2
ALPHA_pos   = 0.2

RAD_PER_CMD = 4.8 / 1000.0
CMD_PER_RAD = 1.0 / RAD_PER_CMD
CMD_MIN, CMD_MAX = -1000, 1000

LEFT_CMD_SIGN  = 1
RIGHT_CMD_SIGN = -1

RELATIVE_INPUT = True




def clamp(x, a, b):
    return a if x < a else (b if x > b else x)

def normalize_angle(a):
    x = (a + pi) % (2 * pi) - pi
    return x if x != -pi else pi

def yaw_from_quat(qx, qy, qz, qw):
    return atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

def wheel_radps_to_cmd(omega):
    return int(clamp(omega * CMD_PER_RAD, CMD_MIN, CMD_MAX))

def bearing_and_distance(x, y, xd, yd):
    dx, dy = xd - x, yd - y
    d = hypot(dx, dy)
    psi_goal = atan2(dy, dx)
    return d, psi_goal




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




class OdomPose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.stamp = 0.0
        self.have_pose = False
        self.lock = threading.Lock()
        self.fx = 0.0
        self.fy = 0.0
        self.fth = 0.0
        self.init_filt = False

    def set_pose(self, x, y, th, stamp):
        with self.lock:
            if not self.init_filt:
                self.fx, self.fy, self.fth = x, y, normalize_angle(th)
                self.init_filt = True
            else:
                self.fx = (1.0 - ALPHA_pos) * self.fx + ALPHA_pos * x
                self.fy = (1.0 - ALPHA_pos) * self.fy + ALPHA_pos * y
                e = normalize_angle(th - self.fth)
                self.fth = normalize_angle(self.fth + ALPHA_yaw * e)
            self.x, self.y, self.th = x, y, normalize_angle(th)
            self.stamp = stamp
            self.have_pose = True

    def get(self, filtered=True):
        with self.lock:
            if filtered and self.init_filt:
                return self.fx, self.fy, self.fth, self.stamp, self.have_pose
            else:
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





# For P-controller

def global_to_wheel_angular_velocity(v, w):
    half_L_over_r = (track_width * 0.5) / wheel_radius
    base = v / wheel_radius
    right_wheel_angular_velocity = base + half_L_over_r * w
    left_wheel_angular_velocity  = base - half_L_over_r * w
    peak = max(abs(left_wheel_angular_velocity), abs(right_wheel_angular_velocity))
    if peak > wheel_w_max:
        s = wheel_w_max / peak
        left_wheel_angular_velocity  *= s
        right_wheel_angular_velocity *= s
    return left_wheel_angular_velocity, right_wheel_angular_velocity

def Wheel_to_servo(motor: Motor, om_l, om_r, swap_lr=False, invert=False):
    left_command  = wheel_radps_to_cmd(om_l)
    right_command = wheel_radps_to_cmd(om_r)
    if invert:
        left_command, right_command = -left_command, -right_command
    left_command  *= LEFT_CMD_SIGN
    right_command *= RIGHT_CMD_SIGN
    if swap_lr:
        left_command, right_command = right_command, left_command
    motor.start([left_command, right_command])

def p_controller_step(x, y, th, xd, yd, thd):
    dx, dy = xd - x, yd - y
    error_x  =  cos(th) * dx + sin(th) * dy
    error_y  = -sin(th) * dx + cos(th) * dy
    error_psi = normalize_angle(thd - th)

    distance_to_goal, psi_goal = bearing_and_distance(x, y, xd, yd)
    error_bearing = normalize_angle(psi_goal - th)

    blend = clamp(1.0 - distance_to_goal / D_blend, 0.0, 1.0)

    v = Kp_x_fwd * error_x
    w = Kp_y_lat * error_y + Kp_psi_yaw * error_psi + (Kp_psi_final * blend * error_psi)

   
    v += Kp_distance * distance_to_goal
    w += Kp_heading_bearing * error_bearing

    v = clamp(v, -v_max, v_max)
    w = clamp(w, -w_max, w_max)

    return v, w, distance_to_goal, abs(error_psi)

def check_goal_reached(d, abs_epsi):
    return (d <= position_deadband) and (abs_epsi <= yaw_deadband)

def plan_from_keyboard():
    raw_input = input().strip()
    dx, dy, dw = map(float, raw_input.split())
    return dx, dy, dw

def p_controller_loop(motor: Motor, pose: OdomPose, x_d, y_d, th_d, swap_lr=False, invert=False, max_time=20.0):
    t0 = time.time()
    while True:
        rclpy.spin_once(p_controller_loop.node, timeout_sec=0.0)
        x, y, th, _, have = pose.get(filtered=True)
        if not have:
            time.sleep(0.01)
            continue

        v, w, distance_to_goal, abs_heading_error = p_controller_step(x, y, th, x_d, y_d, th_d)

        if check_goal_reached(distance_to_goal, abs_heading_error):
            motor.stop()
            return True

        om_l, om_r = global_to_wheel_angular_velocity(v, w)
        motor.update()
        Wheel_to_servo(motor, om_l, om_r, swap_lr, invert)

        if time.time() - t0 > max_time:
            motor.stop()
            return False

        time.sleep(sample_time)




def main():
    rclpy.init(args=None)
    pose = OdomPose()
    node = OdomListener(pose)
    p_controller_loop.node = node
    motor = Motor(servo_ids=[2, 3], window_size=10)
    try:
        t0 = time.time()
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            _, _, _, _, have = pose.get()
            if have:
                break
            if time.time() - t0 > 2.0:
                t0 = time.time()

        while True:
            dx, dy, dth = plan_from_keyboard()
            x, y, th, _, _ = pose.get(filtered=True)
            if RELATIVE_INPUT:
                x_d = x + dx * cos(th) - dy * sin(th)
                y_d = y + dx * sin(th) + dy * cos(th)
                th_d = normalize_angle(th + dth)
            else:
                x_d, y_d, th_d = dx, dy, normalize_angle(dth)

            ok = p_controller_loop(motor, pose, x_d, y_d, th_d, swap_lr=False, invert=False, max_time=20.0)
            if not ok:
                print("Timed out; stopping.")
    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
