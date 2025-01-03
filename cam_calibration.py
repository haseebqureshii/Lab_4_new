from imports import cv2, np, sys
from env_var import ARUCO_DICT, BOARD_SIDE_LENGTH, MARKER_SIDE_LENGTH, CAPTURE_THRESHOLD, CAM_TO_BOARD_LENGTH, NUM_OF_MARKERS

def calibrate_camera_live():
    # Define the ArUco dictionary and ArUco GridBoard
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.GridBoard(
        (5, 5), markerLength=BOARD_SIDE_LENGTH, markerSeparation=MARKER_SIDE_LENGTH, dictionary=aruco_dict
    )
    objpoints, imgpoints = [], []
    all_ids, counters = [], []
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera could not be opened.")
        sys.exit()
    captured_frames = 0

    print("Starting live calibration...")
    while True and captured_frames < CAPTURE_THRESHOLD:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Failed to capture frame.")
            sys.exit()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None and len(ids) >= NUM_OF_MARKERS:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # Collect corners and IDs for calibration
            objpoints.extend(board.getObjPoints())  # 3D points of the board
            imgpoints.extend(corners)  # 2D points from the image
            all_ids.extend(ids)        
            counters.append(len(ids))  
            captured_frames += 1
            percent = (captured_frames / CAPTURE_THRESHOLD) * 100
            bar = "|" * captured_frames + "-" * (CAPTURE_THRESHOLD - captured_frames)
            sys.stdout.write(f"\r{bar} {percent:.1f}%")
            sys.stdout.flush()

    cap.release()
    cv2.destroyAllWindows()
    # Perform camera calibration
    print(" - Done Calibrating... Hold steady")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraAruco(
        corners=imgpoints,
        ids=np.concatenate(all_ids),
        counter=np.array(counters),
        board=board,
        imageSize=gray.shape[::-1],
        cameraMatrix=None,
        distCoeffs=None,
        rvecs=[0, 0, 0],
        tvecs=[0, 0, CAM_TO_BOARD_LENGTH]
    )

    if ret:
        print("Camera matrix:\n", camera_matrix)
        print("Distortion coefficients:\n", dist_coeffs)
        return camera_matrix, dist_coeffs, rvecs, tvecs
    else:
        print("Calibration failed.")
        return None, None