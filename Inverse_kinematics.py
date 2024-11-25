from imports import np, sys, ik, chain
from robot_comms import send_tcp_packet
from env_var import TARGET_ORIENTATION

def calculate_joint_angles(robot_chain: chain, target_position):
    """
    Compute inverse kinematics to find joint angles for the target position.
    :param robot_chain: The robot's kinematic chain (ikpy.Chain).
    :param target_position: Target [x, y, z] position of the end effector in the robot's frame.
    :return: List of joint angles.
    """
    old_position = [0, 0, 0]
    #target_frame = np.eye(4)
    #target_frame[:3, 3] = target_position  # Set target position in 4X4 matrix
    print(f"Target position:\n{target_position}")
    """ curr_angles = send_tcp_packet(f"get_angles()")
    if curr_angles is  [-1.0, -2.0, -3.0, -4.0, -1.0, -1.0]:
        print("Error retrieving current robot angles")
        return [0, 0, 0, 0, 0, 0]
    """
    while True:
        joint_angles = robot_chain.inverse_kinematics(
            target_position, 
            TARGET_ORIENTATION, 
            orientation_mode="Y"
        )
        sys.stdout.write(f"*")
        sys.stdout.flush()
        fk = robot_chain.forward_kinematics(joint_angles)[:3, 3]
        print(fk)
        if np.allclose(fk, target_position, atol=0.05):
            break

    return joint_angles