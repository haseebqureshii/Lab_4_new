import cv2
import numpy as np
import socket
import time
from ikpy.chain import Chain
from ikpy.link import URDFLink

CAMERA_ID = 0  # Zero for webcam, one for any other camera, and so forth
ARUCO_DICT = cv2.aruco.DICT_6X6_1000
ARUCO_1_ID = 0  # ArUco ID while generating
ARUCO_2_ID = 1  # ArUco ID while generating
MARKER_SIDE_LENGTH = 0.03  # ArUco marker side length in meters
BOARD_SIDE_LENGTH = 0.15  # ArUco board side length
SERVER_IP = '192.168.1.159'
SERVER_PORT = 5001
X_OFFSET = -0.704534  # X offset in meters (to measure in the lab)
Y_OFFSET = -0.684993  # Y offset in meters (to measure in the lab)
Z_OFFSET = 0.05  # Desired vertical height off ee from the robot base
CAPTURE_THRESHOLD = 30  # Number of frames to capture
CAM_TO_BOARD_LENGTH = 0.35  # Vertical distance from camera to ArUco board in meters


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
        exit()
    captured_frames = 0

    print("Starting live calibration...")
    while True and captured_frames < CAPTURE_THRESHOLD:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Failed to capture frame.")
            exit()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # Collect corners and IDs for calibration
            objpoints.extend(board.getObjPoints())  # 3D points of the board
            imgpoints.extend(corners)  # 2D points from the image
            all_ids.extend(ids)        
            counters.append(len(ids))  
            captured_frames += 1
            print(f"Frame {captured_frames} captured.")

    cap.release()
    cv2.destroyAllWindows()
    # Perform camera calibration
    print("Calibrating... Hold steady")
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
        print("Calibration successful!")
        print("Camera matrix:\n", camera_matrix)
        print("Distortion coefficients:\n", dist_coeffs)
        return camera_matrix, dist_coeffs, rvecs, tvecs
    else:
        print("Calibration failed.")
        return None, None

def detect_aruco_markers_and_poses():
    # Define the ArUco dictionary and detector parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    parameters = cv2.aruco.DetectorParameters()
    camera_matrix, dist_coeffs, _, _ = calibrate_camera_live()
    marker_length = MARKER_SIDE_LENGTH
    
    # For each detected marker, extract the rotation and translation vectors
    """ for i in range(len(aprvecs)): 
        aprvecs = aprvecs[i].flatten().flatten()
        aptvecs = aptvecs[i].flatten().flatten()
        rvec_tuple = tuple(aprvecs)
        tvec_tuple = tuple(aptvecs) """

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
            tvec_1 = tvecs[marker_1_idx][0]  # Translation vector
            rvec_1 = rvecs[marker_1_idx][0]  # Rotation vector
            # Pose of marker 2
            tvec_2 = tvecs[marker_2_idx][0]
            rvec_2 = rvecs[marker_2_idx][0]
            # Draw axes on the markers for visualization
            cv2.imshow("ArUco Detection", frame)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_1, tvec_1, marker_length)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_2, tvec_2, marker_length)
            cv2.waitKey(2500)
            cap.release()
            cv2.destroyAllWindows()
            # Return translation and rotation vectors for both markers
            return {"marker_1": {"tvec": tvec_1, "rvec": rvec_1},
                    "marker_2": {"tvec": tvec_2, "rvec": rvec_2}}
        
        print("detectMarkers() returned 'None' type ids")
        break

    cap.release()
    cv2.destroyAllWindows()


def send_tcp_packet(client_socket: socket.socket, message: str):
    # Send a message to the server and receive the response.
    try:
        client_socket.sendall(message.encode('utf-8'))
        response = client_socket.recv(1024).decode('utf-8')
        return response
    except socket.error as e:
        print(f"Socket error: {e}")
        return None

def transform_to_robot_frame(tvec_camera: np.int64, transformation_matrix):
    """
    Transform a 3D position from the camera frame to the robot frame.
    :param tvec_camera: Translation vector in camera frame (e.g., [x, y, z]).
    :param transformation_matrix: 4x4 transformation matrix from camera to robot base.
    :return: Transformed position in the robot frame.
    """
    tvec_camera = tvec_camera.flatten()
    tvec_camera_homogeneous = np.array([tvec_camera[0], tvec_camera[1], tvec_camera[2], 1])  # Homogeneous coordinates
    tvec_robot_homogeneous = np.dot(transformation_matrix, tvec_camera_homogeneous)
    return tvec_robot_homogeneous[:3]

def setup_robot_kinematics():
    # Define the robot chain (based on your robot's URDF or specifications)
    robot_chain = Chain(name="mycobot_chain", links=[
        URDFLink(name="base", origin_translation=[0, 0, 0], origin_orientation=[0, 0, 0], joint_type="revolute", rotation=[0, 0, 1]),
        URDFLink(name="joint_1", origin_translation=[0, 0, 0.21], origin_orientation=[0, 0, 0], joint_type="revolute", rotation=[0, 0, 1]),
        URDFLink(name="joint_2", origin_translation=[0, 0, 0.25], origin_orientation=[0, 0, 0], joint_type="revolute", rotation=[0, 1, 0]),
        URDFLink(name="joint_3", origin_translation=[0, 0, 0.25], origin_orientation=[0, 0, 0], joint_type="revolute", rotation=[0, 0, 1]),
        URDFLink(name="joint_4", origin_translation=[0, 0, 0.107], origin_orientation=[0, 0, 0], joint_type="revolute", rotation=[0, 1, 0]),
        URDFLink(name="joint_5", origin_translation=[0, 0, 0.0762], origin_orientation=[0, 0, 0], joint_type="revolute", rotation=[0, 0, 1]),
        URDFLink(name="end_effector", origin_translation=[0, 0, 0.1095], origin_orientation=[0, 0, 0], joint_type="fixed")
    ])
    return robot_chain

def calculate_joint_angles(robot_chain: Chain, target_position):
    """
    Compute inverse kinematics to find joint angles for the target position.
    :param robot_chain: The robot's kinematic chain (ikpy.Chain).
    :param target_position: Target [x, y, z] position of the end effector in the robot's frame.
    :return: List of joint angles.
    """
    target_frame = np.eye(4)
    target_frame[:3, 3] = target_position  # Set target position
    target_position = target_frame[:3, 3]
    print(f"Target position:\n{target_position}")
    joint_angles = robot_chain.inverse_kinematics(target_position=target_position)
    return joint_angles

if __name__ == "__main__":
    # Measure these in the lab:
    transformation_matrix = np.array([
        [1, 0, 0, X_OFFSET],
        [0, 1, 0, Y_OFFSET],
        [0, 0, 1, Z_OFFSET],
        [0, 0, 0, 1]
    ])

    # Step 1: Detect ArUco markers
    marker_poses = detect_aruco_markers_and_poses()
    if marker_poses is None:
        print("Something went wrong in detect_aruco_markers_and_poses()")

    # Step 2: Transform marker positions to robot frame
    tvec_1_robot = transform_to_robot_frame(marker_poses["marker_1"]["tvec"], transformation_matrix)
    tvec_2_robot = transform_to_robot_frame(marker_poses["marker_2"]["tvec"], transformation_matrix)

    # Step 3: Setup robot kinematics
    robot_chain = setup_robot_kinematics()

    # Step 4: Calculate joint angles
    angles_1 = calculate_joint_angles(robot_chain, tvec_1_robot)
    angles_2 = calculate_joint_angles(robot_chain, tvec_2_robot)

    print("Code execution finished\n" + "__________Results__________")
    print(f"Movement angles (radians) for marker 1: {angles_1}")
    print(f"Movement angles (radians) for marker 2: {angles_2}")

    # Step 5: Move robot to target positions, uncomment when connected to robot
    """ with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print("Moving to Marker 1")
        send_tcp_packet(client_socket, f"set_angles({', '.join(map(str, angles_1))}, 600)")
        time.sleep(4)

        print("Moving to Marker 2")
        send_tcp_packet(client_socket, f"set_angles({', '.join(map(str, angles_2))}, 600)")
        time.sleep(4) """