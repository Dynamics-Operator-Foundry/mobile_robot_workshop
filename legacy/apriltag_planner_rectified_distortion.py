#!/usr/bin/env python3
import time, threading, cv2, apriltag, numpy as np
from math import pi, atan2, hypot, sin, cos
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Odometry
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf_transformations import quaternion_from_matrix
from common.lx16a import LX16A

class DualAprilTargetRect(Node):
    def __init__(self):
        super().__init__('dual_apriltag_target_rectified')

        # --- params ---
        self.image_topic = self.declare_parameter('image_topic', '/camera/fisheye1/image_raw').get_parameter_value().string_value
        self.tag_size = float(self.declare_parameter('tag_size_m', 0.0625).value)   # 6.25 cm
        self.offset_m = float(self.declare_parameter('offset_m', 0.10).value)       # 10 cm
        self.id0 = int(self.declare_parameter('id0', 0).value)
        self.id1 = int(self.declare_parameter('id1', 1).value)
        self.frame_id = self.declare_parameter('frame_id', 'camera_fisheye1_optical_frame').get_parameter_value().string_value

        # --- your fisheye intrinsics from /camera/fisheye1/camera_info (screenshot) ---
        self.K_fish = np.array([[284.1806302734375, 0.0, 425.24481201171875],
                                [0.0, 285.1946105957031, 398.6476135253906],
                                [0.0, 0.0, 1.0]], dtype=np.float64)
        self.D_fish = np.array([0.0003750845, 0.0287880, -0.0290440, 0.008344468], dtype=np.float64)  # k1..k4 (equidistant)

        # Rectify to same K & size
        self.rect_K = self.K_fish.copy()
        self.map1 = None; self.map2 = None
        self.bridge = CvBridge()
        self.det = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))

        self.pub_pose  = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.pub_debug = self.create_publisher(Image, '/apriltag_debug', 1)
        self.create_subscription(Image, self.image_topic, self.cb_image, 10)

        s = self.tag_size/2.0
        self.obj_pts = np.array([[-s, s, 0.0],[ s, s, 0.0],[ s,-s, 0.0],[-s,-s, 0.0]], np.float32)

    def ensure_rectification(self, shape):
        h, w = shape[:2]
        if self.map1 is not None and self.map1.shape[:2]==(h,w): return
        R = np.eye(3)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K_fish, self.D_fish, R, self.rect_K, (w, h), cv2.CV_16SC2
        )

    def cb_image(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.ensure_rectification(bgr.shape)
        rect = cv2.remap(bgr, self.map1, self.map2, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)

        dets = self.det.detect(gray)
        need = {self.id0: None, self.id1: None}
        for d in dets:
            if d.tag_id in need: need[d.tag_id] = d
            c = d.corners.astype(int)
            for i in range(4): cv2.line(rect, tuple(c[i]), tuple(c[(i+1)%4]), (0,255,0), 2)

        if any(v is None for v in need.values()):
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(rect, 'bgr8')); return

        K = self.rect_K.astype(np.float64); D0 = np.zeros(4)
        centers, normals, Rs = [], [], []
        for tid in [self.id0, self.id1]:
            d = need[tid]
            img_pts = np.array(d.corners, np.float32)
            ok, rvec, tvec = cv2.solvePnP(self.obj_pts, img_pts, K, D0, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok: return
            R,_ = cv2.Rodrigues(rvec)
            Rs.append(R)
            centers.append(tvec.reshape(3))
            n = R[:,2]; normals.append(n/(np.linalg.norm(n)+1e-9))

        c_mid = 0.5*(centers[0]+centers[1])
        n_avg = normals[0]+normals[1]; n_avg /= (np.linalg.norm(n_avg)+1e-9)
        p_target = c_mid - self.offset_m*n_avg  # 10 cm toward camera

        x0 = Rs[0][:,0]
        x_proj = x0 - np.dot(x0, n_avg)*n_avg
        x_hat = x_proj/(np.linalg.norm(x_proj)+1e-9)
        y_hat = np.cross(n_avg, x_hat); y_hat /= (np.linalg.norm(y_hat)+1e-9)
        R_cam = np.column_stack([x_hat, y_hat, n_avg])

        T = np.eye(4); T[:3,:3]=R_cam; T[:3,3]=p_target
        q = quaternion_from_matrix(T)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'camera_fisheye1_optical_frame'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, p_target.tolist())
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q.tolist())
        self.pub_pose.publish(pose)

        u,v = self.project(K, p_target)
        if u is not None:
            cv2.circle(rect, (u,v), 6, (255,0,0), -1)
            cv2.putText(rect, 'target 10cm', (u+8, v-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(rect, 'bgr8'))

    def project(self, K, P):
        x,y,z = P
        if z <= 1e-6: return (None, None)
        u = K[0,0]*x/z + K[0,2]
        v = K[1,1]*y/z + K[1,2]
        return int(round(u)), int(round(v))

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
RELATIVE_INPUT = True

def normalize_angle(a):
    x = (a + pi) % (2 * pi) - pi
    return x if x != -pi else pi

def wheel_radps_to_cmd(omega): return int(np.clip(omega * CMD_PER_RAD, CMD_MIN, CMD_MAX))
def yaw_from_quat(qx, qy, qz, qw): return atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

class Motor:
    def __init__(self, servo_ids=[2,3], port="/dev/ttyUSB0", window_size=10):
        LX16A.initialize(port, 0.1)
        self.servos = [LX16A(sid) for sid in servo_ids]
        for s in self.servos: s.motor_mode(0)
        self.window_size = window_size
        self.timestamps = np.zeros(window_size)
        self.unwrapped_angles = np.zeros((window_size, len(self.servos)))
        self.raw_angles = np.zeros(len(self.servos))
        self.prev_angles = np.zeros(len(self.servos))
        self.motor_vel = np.zeros(len(self.servos))
        self.target_speeds = np.zeros(len(self.servos), dtype=int)
        self.sample_idx = 0; self.is_initialized=False

    def start(self, speeds):
        speeds = np.array(speeds, dtype=int); self.target_speeds = speeds
        for i, s in enumerate(self.servos): s.motor_mode(int(speeds[i]))

    def stop(self):
        self.target_speeds[:] = 0
        for s in self.servos: s.motor_mode(0)

    def update(self):
        for i, s in enumerate(self.servos): self.raw_angles[i] = s.get_physical_angle()
        if self.sample_idx == 0 and not self.is_initialized:
            unwrapped = self.raw_angles.copy()
        else:
            delta = self.raw_angles - self.prev_angles
            delta[delta < -180] += 360; delta[delta > 180] -= 360
            prev_idx = (self.sample_idx - 1) % self.window_size
            unwrapped = self.unwrapped_angles[prev_idx] + delta
        self.timestamps[self.sample_idx] = time.time()
        self.unwrapped_angles[self.sample_idx] = unwrapped
        self.prev_angles = self.raw_angles.copy()
        self.sample_idx = (self.sample_idx + 1) % self.window_size
        if not self.is_initialized and self.sample_idx == 0: self.is_initialized=True
        if self.is_initialized:
            oldest = self.sample_idx; newest = (self.sample_idx - 1) % self.window_size
            dt = self.timestamps[newest] - self.timestamps[oldest]
            if dt > 0:
                self.motor_vel = (self.unwrapped_angles[newest] - self.unwrapped_angles[oldest]) * (pi/180.0) / dt

    def get_vel(self): return self.motor_vel.copy()

class OdomPose:
    def __init__(self):
        self.x=self.y=self.th=self.stamp=0.0; self.have_pose=False
        self.lock = threading.Lock()
    def set_pose(self, x,y,th,stamp):
        with self.lock: self.x,self.y,self.th,self.stamp,self.have_pose = x,y,normalize_angle(th),stamp,True
    def get(self):
        with self.lock: return self.x,self.y,self.th,self.stamp,self.have_pose

class Rover(Node):
    def __init__(self, pose_store: OdomPose):
        super().__init__('rover_controller')
        self.pose_store = pose_store
        self.have_target=False; self.dx=self.dy=self.dth=0.0
        self.tlock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self.create_subscription(Odometry, '/camera/odom/sample', self.odom_cb, qos)
        self.create_subscription(Odometry, '/camera/pose/sample', self.odom_cb, qos)
        self.create_subscription(PoseStamped, '/target_pose', self.target_cb, 10)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        th = yaw_from_quat(q.x,q.y,q.z,q.w)
        self.pose_store.set_pose(p.x, p.y, th, msg.header.stamp.sec + 1e-9*msg.header.stamp.nanosec)

    def target_cb(self, msg: PoseStamped):
        # Optical (x right, y down, z forward) -> Base (x fwd, y left, z up):
        xo, yo, zo = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        xb, yb = zo, -xo  # (zb not used)
        # Orientation: apply fixed rotation R_ob
        qw,qx,qy,qz = msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
        Ropt = np.array([
            [1-2*(qy*qy+qz*qz),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
            [  2*(qx*qy + qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz - qx*qw)],
            [  2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw), 1-2*(qx*qx+qy*qy)]
        ], dtype=float)
        R_ob = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=float)
        Rb = R_ob @ Ropt
        yaw_b = atan2(Rb[1,0], Rb[0,0])
        with self.tlock:
            self.dx, self.dy, self.dth = float(xb), float(yb), float(yaw_b)
            self.have_target = True

    def get_target(self):
        with self.tlock: return self.dx, self.dy, self.dth, self.have_target

def compute_wheel_radps_from_vw(v, w):
    v_l = v - w * half_track_width
    v_r = v + w * half_track_width
    om_l = np.clip(v_l / wheel_radius, -max_angular_speed, max_angular_speed)
    om_r = np.clip(v_r / wheel_radius, -max_angular_speed, max_angular_speed)
    return om_l, om_r

def send_wheels(motor: LX16A, om_l, om_r, swap_lr=False, invert=False):
    lc = wheel_radps_to_cmd(om_l); rc = wheel_radps_to_cmd(om_r)
    if invert: lc, rc = -lc, -rc
    lc *= LEFT_CMD_SIGN; rc *= RIGHT_CMD_SIGN
    if swap_lr: lc, rc = rc, lc
    motor.start([lc, rc])

def p_go_to(motor, pose_store: OdomPose, node_handle: Node,
            x_d, y_d, th_d, Kp_lin=0.6, Kp_ang=1.2, dt=0.05,
            stop_r=0.02, stop_th=4.0*pi/180.0, swap_lr=False, invert=False, max_time=15.0):
    t0 = time.time()
    # phase 1: turn toward goal
    while True:
        rclpy.spin_once(node_handle, timeout_sec=0.0)
        x,y,th,_,have = pose_store.get()
        if not have: time.sleep(0.02); continue
        ex,ey = x_d-x, y_d-y
        if hypot(ex,ey) < 1e-3: break
        err = normalize_angle(atan2(ey,ex)-th)
        if abs(err) <= stop_th: break
        v=0.0; w=float(np.clip(Kp_ang*err, -W_MAX, W_MAX))
        if abs(w) < TH_DEADBAND: w=0.0
        om_l,om_r = compute_wheel_radps_from_vw(v,w)
        motor.update(); send_wheels(motor, om_l, om_r, swap_lr, invert)
        if time.time()-t0 > max_time: motor.stop(); return False
        time.sleep(dt)
    # phase 2: go-to-goal
    while True:
        rclpy.spin_once(node_handle, timeout_sec=0.0)
        x,y,th,_,have = pose_store.get()
        if not have: time.sleep(0.02); continue
        ex,ey = x_d-x, y_d-y
        epos = hypot(ex,ey)
        if epos <= stop_r: motor.stop(); break
        theta_path = atan2(ey,ex)
        heading_err = normalize_angle(theta_path-th)
        v = float(np.clip(Kp_lin*epos, -V_MAX, V_MAX))
        w = float(np.clip(Kp_ang*heading_err, -W_MAX, W_MAX))
        if abs(w) < TH_DEADBAND: w=0.0
        om_l,om_r = compute_wheel_radps_from_vw(v,w)
        motor.update(); send_wheels(motor, om_l, om_r, swap_lr, invert)
        if time.time()-t0 > max_time: motor.stop(); return False
        time.sleep(dt)
    # phase 3: align yaw
    while True:
        rclpy.spin_once(node_handle, timeout_sec=0.0)
        x,y,th,_,have = pose_store.get()
        if not have: time.sleep(0.02); continue
        eth = normalize_angle(th_d-th)
        if abs(eth) <= stop_th: motor.stop(); return True
        v=0.0; w=float(np.clip(Kp_ang*eth, -W_MAX, W_MAX))
        if abs(w) < TH_DEADBAND: w=0.0
        om_l,om_r = compute_wheel_radps_from_vw(v,w)
        motor.update(); send_wheels(motor, om_l, om_r, swap_lr, invert)
        if time.time()-t0 > max_time: motor.stop(); return False
        time.sleep(dt)

def main():
    rclpy.init()

    # Nodes
    tag_node = DualAprilTargetRect()
    pose_store = OdomPose()
    rover = Rover(pose_store)
    motor = Motor(servo_ids=[2,3], window_size=10)

    exec = MultiThreadedExecutor(num_threads=2)
    exec.add_node(tag_node)
    exec.add_node(rover)

    try:
        # Warm up: wait for initial odom
        t0 = time.time()
        while True:
            exec.spin_once(timeout_sec=0.1)
            _,_,_,_,have = pose_store.get()
            if have: break
            if time.time()-t0 > 2.0: t0 = time.time()

        # Main loop: wait for /target_pose, convert to world goal, drive
        while rclpy.ok():
            exec.spin_once(timeout_sec=0.05)
            dx, dy, dth, have_t = rover.get_target()
            if not have_t: continue

            x,y,th,_,_ = pose_store.get()
            if RELATIVE_INPUT:
                x_d = x + dx * cos(th) - dy * sin(th)
                y_d = y + dx * sin(th) + dy * cos(th)
                th_d = normalize_angle(th + dth)
            else:
                x_d, y_d, th_d = dx, dy, dth

            p_go_to(motor, pose_store, rover,
                    x_d, y_d, th_d,
                    Kp_lin=0.6, Kp_ang=1.2, dt=0.05,
                    stop_r=0.02, stop_th=4.0*pi/180.0,
                    swap_lr=False, invert=False, max_time=20.0)

    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        exec.shutdown()
        tag_node.destroy_node(); rover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
