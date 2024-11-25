import math
from imports import np, socket, time
from env_var import X_OFFSET, Y_OFFSET, Z_OFFSET, EE_VELOCITY
from aruco_detection import detect_aruco_markers_and_poses
from cam_robot_transform import transform_to_robot_frame
from kinematics_setup import setup_robot_kinematics
from Inverse_kinematics import calculate_joint_angles
from robot_comms import send_tcp_packet

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
    print("Movement angles (degrees) for marker 1: ", list(map(lambda r:math.degrees(r), angles_1)))
    print("Movement angles (degrees) for marker 2: ", list(map(lambda r:math.degrees(r), angles_1)))
    #print(f"Movement angles (radians) for marker 1: {angles_1}")
    #print(f"Movement angles (radians) for marker 2: {angles_2}")

    # Step 5: Move robot to target positions, uncomment when connected to robot
    """ print("Moving to Marker 1")
    send_tcp_packet(f"set_angles({', '.join(map(str, angles_1))}, {EE_VELOCITY})")
    time.sleep(3)

    print("Moving to Marker 2")
    send_tcp_packet(f"set_angles({', '.join(map(str, angles_2))}, {EE_VELOCITY})")
    time.sleep(3) """