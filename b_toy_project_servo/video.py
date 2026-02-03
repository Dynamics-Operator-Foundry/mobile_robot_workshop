#!/usr/bin/env python3

import cv2
import apriltag

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped

class AprilTagPoseEstimator(Node):
    def __init__(self, camera_matrix, dist_coeffs, tag_size, target_id):
        super().__init__('apriltag_pose_estimator')
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.tag_size = tag_size
        self.target_id = target_id
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag25h9"))
        # self.tracked_points = []
        # self.poses = []
        # self.bridge = CvBridge()
        # self.pose_pub = self.create_publisher(PoseStamped, '/mavros/mocap/pose', 10)
        # self.image_pub = self.create_publisher(Image, '/processed_image', 10)
        # self.image_sub = self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.image_callback, 10)
        
        cap = cv2.VideoCapture(0)  # 0 = default camera
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            raise RuntimeError("Could not open camera")

        for _ in range(1000):  # number of frames to capture
            ret, frame = cap.read()
            
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame
            gray = gray.astype('uint8')
            
            detections = self.detector.detect(gray)
            
            # success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)
            
            print(len(detections))
            
            if not ret:
                print("Failed to grab frame")
                break

            cv2.imshow("Camera", frame)

            # press 'q' to quit early
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        cap.release()
        cv2.destroyAllWindows()

    

        
        
        

def main(args=None):
    rclpy.init(args=args)

    camera_matrix = np.array([
        [628.8233032226562, 0, 646.732666015625],
        [0, 628.3672485351562, 364.4508361816406],
        [0, 0, 1]
    ], dtype=np.float32)

    # -0.056777212768793106, 0.06796900182962418, 0.0007022436475381255, 0.0004860123444814235, -0.021817076951265335

    dist_coeffs = np.array([-0.056777212768793106, 0.06796900182962418, 0.0007022436475381255, 0.0004860123444814235], dtype=np.float32)
    tag_size = 0.3
    target_id = 0

    estimator = AprilTagPoseEstimator(camera_matrix, dist_coeffs, tag_size, target_id)
    
    try:
        rclpy.spin(estimator)
    except KeyboardInterrupt:
        pass
    finally:
        estimator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()