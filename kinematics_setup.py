from imports import chain

def setup_robot_kinematics():
    # Define the robot chain (based on your robot's URDF or specifications)
    robot_chain = chain.Chain.from_urdf_file("myCobot_Pro_600.urdf", active_links_mask=[False, True, True, True, True, True, True])
    return robot_chain