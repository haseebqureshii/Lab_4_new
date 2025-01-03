from imports import np, sys, ik, chain, time
from robot_comms import send_tcp_packet
from plot_chain import plot_robot_chain
from env_var import TARGET_ORIENTATION

def calculate_joint_angles(robot_chain: chain, target_position):
    """
    Compute inverse kinematics to find joint angles for the target position.
    :param robot_chain: The robot's kinematic chain (ikpy.Chain).
    :param target_position: Target [x, y, z] position of the end effector in the robot's frame.
    :return: List of joint angles.
    """
    old_position = [0, -1.5708, 0, 0, -1.5708, 0] # fetch robot angles here
    target_frame = np.eye(4)
    target_frame[:3, 3] = target_position  # Set target position in 4X4 matrix
    print(f"Target position:\n{target_position}")
    """ curr_angles = send_tcp_packet(f"get_angles()")
    if curr_angles is  [-1.0, -2.0, -3.0, -4.0, -1.0, -1.0]:
        print("Error retrieving current robot angles")
        return [0, 0, 0, 0, 0, 0]
    """
    
    #while True:
    joint_angles = robot_chain.inverse_kinematics(
        target_position, 
        TARGET_ORIENTATION, 
        orientation_mode=None,
    )
    plot_robot_chain(joint_angles, robot_chain, target_position)
    time.sleep(30)
    """ joint_angles = ik.inverse_kinematic_optimization(
        robot_chain,
        target_frame,
        old_position,
        0.05,
        1000,
        "Z"
    ) """
    #sys.stdout.write(f"*")
    #sys.stdout.flush()
    fk = robot_chain.forward_kinematics(joint_angles)[:3, 3]
    print(f"Forward Kinematics:\n {fk}")
    #if np.allclose(fk, target_position, atol=0.1):
    #break

    return joint_angles