from imports import cv2, np
from env_var import ARUCO_DICT, ARUCO_1_ID, MARKER_SIDE_LENGTH, ARUCO_2_ID
from cam_calibration import calibrate_camera_live

def detect_aruco_markers_and_poses():
    # Define the ArUco dictionary and detector parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    parameters = cv2.aruco.DetectorParameters()
    camera_matrix, dist_coeffs, _, _ = calibrate_camera_live()
    marker_length = MARKER_SIDE_LENGTH

    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            # Extract poses for markers 1 and 2
            marker_1_idx = np.where(ids == ARUCO_1_ID)[0]
            marker_2_idx = np.where(ids == ARUCO_2_ID)[0]
            # Pose of marker 1
            if (len(marker_1_idx) > 0 and len(marker_2_idx) > 0 and
                marker_1_idx[0] < len(tvecs) and marker_1_idx[0] < len(rvecs) and
                marker_2_idx[0] < len(tvecs) and marker_2_idx[0] < len(rvecs)
            ):
                tvec_1 = tvecs[marker_1_idx][0]  # Translation vector
                rvec_1 = rvecs[marker_1_idx][0]  # Rotation vector
                # Pose of marker 2
                tvec_2 = tvecs[marker_2_idx][0]
                rvec_2 = rvecs[marker_2_idx][0]
                # Draw axes on the markers for visualization
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_1, tvec_1, marker_length)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_2, tvec_2, marker_length)
                cv2.imshow("ArUco Detection", frame)
                cv2.waitKey(3000)
                cap.release()
                cv2.destroyAllWindows()
                # Return translation and rotation vectors for both markers
                return {"marker_1": {"tvec": tvec_1, "rvec": rvec_1},
                        "marker_2": {"tvec": tvec_2, "rvec": rvec_2}}
        
        else:
            print("detectMarkers() returned 'None' type ids")
            break

    cap.release()
    cv2.destroyAllWindows()