# t265_apriltag_center_pose.py
import math
import numpy as np
import cv2

# ---------- Performance knobs ----------
DRAW = True                 # set False for max FPS (no overlays)
UNDISTORT_BALANCE = 0.0     # 0..1 (trade FOV vs. black borders)
AXIS_LEN = 0.05             # drawn axis length (meters) if DRAW is True

# ---------- Board geometry ----------
TAG_SIZE_M = 0.0625         # 62.5 mm
EDGE_GAP_M = 0.050          # 50 mm edge-to-edge
TAG_IDS = [0, 1]            # 0 = left, 1 = right
BASELINE_M = TAG_SIZE_M + EDGE_GAP_M      # center-to-center distance
HALF_BASELINE = BASELINE_M * 0.5

# ---------- T265 Fisheye #1 intrinsics ----------
W, H = 848, 800
K_FISH = np.array([
    [284.18060302734375, 0.0,                425.24481201171875],
    [0.0,                285.1946105957031,  398.6476135253906 ],
    [0.0,                  0.0,                1.0]
], dtype=np.float64)
# equidistant (fisheye) k1..k4
D_FISH = np.array([[ 0.0003750845143533498,
                     0.028789799754881911,
                    -0.020044402920991898,
                     0.003844684180717612 ]], dtype=np.float64)

# ---------- Utils ----------
def rvec_to_ypr_deg(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = (R[0,0]**2 + R[1,0]**2) ** 0.5
    if sy > 1e-6:
        yaw   = math.degrees(math.atan2(R[1,0], R[0,0]))
        pitch = math.degrees(math.atan2(-R[2,0], sy))
        roll  = math.degrees(math.atan2(R[2,1], R[2,2]))
    else:
        yaw   = math.degrees(math.atan2(-R[0,1], R[1,1]))
        pitch = math.degrees(math.atan2(-R[2,0], sy))
        roll  = 0.0
    return yaw, pitch, roll

def build_board(dictionary):
    L = TAG_SIZE_M
    centers = {
        TAG_IDS[0]: np.array([-HALF_BASELINE, 0.0, 0.0], dtype=np.float32),
        TAG_IDS[1]: np.array([+HALF_BASELINE, 0.0, 0.0], dtype=np.float32),
    }
    h = L * 0.5
    def corners_at(c):
        cx, cy, cz = c
        return np.array([
            [cx - h, cy + h, cz],   # TL
            [cx + h, cy + h, cz],   # TR
            [cx + h, cy - h, cz],   # BR
            [cx - h, cy - h, cz],   # BL
        ], dtype=np.float32)

    obj_pts = [corners_at(centers[TAG_IDS[0]]), corners_at(centers[TAG_IDS[1]])]
    ids = np.array(TAG_IDS, dtype=np.int32)
    return cv2.aruco.Board_create(obj_pts, dictionary, ids)

def make_undistort_maps(K_fish, D_fish, size, balance=0.0):
    R = np.eye(3)
    Knew = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K_fish, D_fish, size, R, balance=balance, fov_scale=1.0
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K_fish, D_fish, R, Knew, size, cv2.CV_16SC2
    )
    return Knew, map1, map2

def main():
    # OpenCV perf flags
    cv2.setUseOptimized(True)
    try:
        cv2.setNumThreads(0)  # leave cores for the detector; adjust if you prefer
    except Exception:
        pass

    # ---- Detector (Apriltag 36h11 via ArUco) ----
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters_create()
    # speed-oriented tweaks (trade a bit of robustness for FPS)
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.02
    params.maxMarkerPerimeterRate = 4.0

    detector = cv2.aruco.ArucoDetector(dictionary, params) \
        if hasattr(cv2.aruco, "ArucoDetector") else None

    board = build_board(dictionary)

    # ---- RealSense T265 (fisheye #1) ----
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.fisheye, 1, W, H, rs.format.y8, 30)
    prof = pipe.start(cfg)

    # ---- Fisheye -> pinhole maps (precompute once) ----
    Knew, map1, map2 = make_undistort_maps(K_FISH, D_FISH, (W, H), balance=UNDISTORT_BALANCE)
    D_PINHOLE = np.zeros((5, 1), dtype=np.float64)

    last_rvec = None
    last_tvec = None

    print("Press 'q' to quit.")
    # ======== SINGLE RUNTIME LOOP ========
    while True:
        frames = pipe.wait_for_frames()
        f1 = frames.get_fisheye_frame(1)
        if f1 is None:
            continue

        # acquire & undistort (grayscale)
        img = np.asanyarray(f1.get_data())
        rect = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

        # detect
        if detector is not None:
            corners, ids, _ = detector.detectMarkers(rect)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(rect, dictionary, parameters=params)

        # filter to our two IDs without extra loops
        rvec_center = tvec_center = None
        if ids is not None and len(ids) > 0:
            mask = np.isin(ids.ravel(), TAG_IDS)
            if mask.any():
                fc = [corners[i] for i in np.nonzero(mask)[0]]
                fids = ids[mask]
                valid, rvec, tvec = cv2.aruco.estimatePoseBoard(
                    fc, fids, board, Knew, D_PINHOLE, None, None
                )
                if valid > 0:
                    rvec_center, tvec_center = rvec, tvec
                    last_rvec, last_tvec = rvec, tvec

        # visualize only if requested (saves time)
        if DRAW:
            vis = cv2.cvtColor(rect, cv2.COLOR_GRAY2BGR)
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            if rvec_center is not None:
                cv2.drawFrameAxes(vis, Knew, D_PINHOLE, rvec_center, tvec_center, AXIS_LEN)
                yaw, pitch, roll = rvec_to_ypr_deg(rvec_center)
                tx, ty, tz = tvec_center.flatten().tolist()
                text = f"t=[{tx:+.3f},{ty:+.3f},{tz:+.3f}] m | ypr=[{yaw:+.1f},{pitch:+.1f},{roll:+.1f}]°"
                cv2.putText(vis, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)
            cv2.imshow("T265 AprilTag board (center pose)", vis)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
        else:
            # headless: print pose when available and still allow quit
            if rvec_center is not None:
                yaw, pitch, roll = rvec_to_ypr_deg(rvec_center)
                tx, ty, tz = tvec_center.flatten().tolist()
                print(f"t=({tx:.3f},{ty:.3f},{tz:.3f}) m | ypr=({yaw:.1f},{pitch:.1f},{roll:.1f}) deg")
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break

    try: pipe.stop()
    except Exception: pass
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
