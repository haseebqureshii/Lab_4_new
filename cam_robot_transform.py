from imports import np

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