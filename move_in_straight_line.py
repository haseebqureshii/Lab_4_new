from env_var import EE_VELOCITY, TRAJECTORY_STEP_SIZE
from imports import np

from Inverse_kinematics import calculate_joint_angles
from robot_comms import send_tcp_packet


def interpolate_cartesian_path(start_pos, end_pos, steps=TRAJECTORY_STEP_SIZE):
    """
    Generate intermediate waypoints between two Cartesian positions.
    :param start_pos: Starting position (x, y, z).
    :param end_pos: Ending position (x, y, z).
    :param steps: Number of waypoints.
    :return: List of interpolated Cartesian positions.
    """
    return [start_pos + t * (end_pos - start_pos) for t in np.linspace(0, 1, steps)]

def move_in_straight_line(robot_chain, start_pos, end_pos, steps=10):
    waypoints = interpolate_cartesian_path(start_pos, end_pos, steps)
    for point in waypoints:
        joint_angles = calculate_joint_angles(robot_chain, point)
        print(f"Moving to: {point}, Joint Angles: {joint_angles}")
        # Command the robot to move to joint_angles
        send_tcp_packet(f"set_angles({', '.join(map(str, joint_angles))}, {EE_VELOCITY})")
