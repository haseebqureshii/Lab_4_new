import cv2
import numpy as np
import socket
from ikpy.chain import Chain
from ikpy.link import URDFLink

MARKER_SIDE_LENGTH = 0.03  # ArUco marker side length in meters
SERVER_IP = '192.168.1.159'
SERVER_PORT = 5001
X_OFFSET = -0.704534  # X offset in meters (to measure in the lab)
Y_OFFSET = -0.684993  # Y offset in meters (to measure in the lab)
CAPTURE_THRESHOLD = 30  # Number of frames to capture

def detect_aruco_markers_and_poses():
    # Define the ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
    camera_matrix, dist_coeffs = calibrate_camera_live()
    marker_length = MARKER_SIDE_LENGTH

    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            continue
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            ids = ids.flatten()
            if 1 in ids and 2 in ids:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                # Estimate pose for each marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_length, camera_matrix, dist_coeffs)
                
                # Extract poses for markers 1 and 2
                marker_1_idx = np.where(ids == 1)[0][0]
                marker_2_idx = np.where(ids == 2)[0][0]

                tvec_1, rvec_1 = tvecs[marker_1_idx][0], rvecs[marker_1_idx][0]
                tvec_2, rvec_2 = tvecs[marker_2_idx][0], rvecs[marker_2_idx][0]

                # Draw axes on the markers for visualization
                cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec_1, tvec_1, marker_length)
                cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec_2, tvec_2, marker_length)
                
                cv2.imshow("ArUco Detection", frame)
                cv2.waitKey(2000)
                cap.release()
                cv2.destroyAllWindows()

                return {
                    "marker_1": {"tvec": tvec_1, "rvec": rvec_1},
                    "marker_2": {"tvec": tvec_2, "rvec": rvec_2}
                }
        
        cv2.imshow("ArUco Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def calibrate_camera_live():
    # Define the ArUco dictionary and Charuco board
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard(size=(5, 5), squareLength=0.15, markerLength=0.03, dictionary=aruco_dict)
    objpoints, imgpoints = [], []
    captured_frames = 0
    cap = cv2.VideoCapture(0)
    print("Starting live calibration...")

    while cap.isOpened() and captured_frames < CAPTURE_THRESHOLD:
        ret, frame = cap.read()
        if not ret:
            continue
        cv2.imshow("Camera Feed - Calibration", frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            charucodetector = cv2.aruco.CharucoDetector(board)
            charuco_corners, charuco_ids, marker_corners, marker_ids = charucodetector.detectBoard(frame)
            if charuco_corners is not None and charuco_ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)
                imgpoints.append(charuco_corners)
                objpoints.append(board.getChessboardCorners())
                captured_frames += 1
                print(f"Frame {captured_frames} captured.")

    cap.release()
    cv2.destroyAllWindows()

    # Camera calibration using Charuco corners
    ret, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=imgpoints,
        charucoIds=charuco_ids,
        board=board,
        imageSize=gray.shape[::-1],
        cameraMatrix=None,
        distCoeffs=None
    )

    if ret:
        print("Calibration successful!")
        print("Camera Matrix:\n", camera_matrix)
        print("Distortion Coefficients:\n", dist_coeffs)
        return camera_matrix, dist_coeffs
    else:
        print("Calibration failed.")
        return None, None

def transform_to_robot_frame(tvec_camera, transformation_matrix):
    tvec_camera_homogeneous = np.array([*tvec_camera, 1])  # Homogeneous coordinates
    tvec_robot_homogeneous = np.dot(transformation_matrix, tvec_camera_homogeneous)
    return tvec_robot_homogeneous[:3]

def setup_robot_kinematics():
    robot_chain = Chain(name="mycobot_chain", links=[
        URDFLink("base", [0, 0, 0], [0, 0, 0]),
        URDFLink("joint_1", [0, 0, 0.21], [0, 0, 1]),
        URDFLink("joint_2", [0, 0, 0.25], [0, 1, 0]),
        URDFLink("joint_3", [0, 0, 0.25], [0, 0, 1]),
        URDFLink("joint_4", [0, 0, 0.1], [0, 1, 0]),
        URDFLink("joint_5", [0, 0, 0.07], [0, 0, 1]),
        URDFLink("end_effector", [0, 0, 0.05], [0, 0, 0]),
    ])
    return robot_chain

def calculate_joint_angles(robot_chain, target_position):
    target_frame = np.eye(4)
    target_frame[:3, 3] = target_position
    joint_angles = robot_chain.inverse_kinematics(target_frame)
    return joint_angles

if __name__ == "__main__":
    transformation_matrix = np.array([
        [1, 0, 0, X_OFFSET],
        [0, 1, 0, Y_OFFSET],
        [0, 0, 1, 0.05],
        [0, 0, 0, 1]
    ])
    marker_poses = detect_aruco_markers_and_poses()
    robot_chain = setup_robot_kinematics()
    for marker, pose in marker_poses.items():
        tvec_robot = transform_to_robot_frame(pose["tvec"], transformation_matrix)
        angles = calculate_joint_angles(robot_chain, tvec_robot)
        print(f"{marker} angles: {angles}")
    
    # Step 5: Move robot to target positions
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print("Moving to Marker 1")
        send_tcp_packet(client_socket, f"set_angles({', '.join(map(str, angles_1))}, 600)")
        time.sleep(4)

        print("Moving to Marker 2")
        send_tcp_packet(client_socket, f"set_angles({', '.join(map(str, angles_2))}, 600)")
        time.sleep(4)