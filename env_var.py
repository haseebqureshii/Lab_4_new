import cv2
import numpy as np

CAMERA_ID = 1  # 0 = webcam, 1 = plugged in camera
NUM_OF_MARKERS = 2  # No. of ArUco markers to detect
ARUCO_DICT = cv2.aruco.DICT_6X6_1000
ARUCO_1_ID = 1  # ArUco ID while generating
ARUCO_2_ID = 2  # ArUco ID while generating
MARKER_SIDE_LENGTH = 0.025  # ArUco marker side length in meters
BOARD_SIDE_LENGTH = 0.15  # ArUco board side length
SERVER_IP = '192.168.1.159'
SERVER_PORT = 5001
EE_VELOCITY = 600
TARGET_ORIENTATION = [0, 0, -1] # Points downwards (negetive Z direction)
TRAJECTORY_STEP_SIZE = 20  # No. of step i
# For best results, place camera stand and ArUco board in regions that are easy for robot to reach
X_OFFSET = -0.25  # X offset in meters (measure in the lab)
Y_OFFSET = -0.30  # Y offset in meters (measure in the lab)
Z_OFFSET = 0.03  # Desired vertical height off ee from the robot base
CAPTURE_THRESHOLD = 30  # Number of frames to capture
CAM_TO_BOARD_LENGTH = 0.40  # Vertical distance from camera to ArUco board in meters