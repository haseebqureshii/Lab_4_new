
# ArUco-Based Robot Control with Inverse Kinematics

This project demonstrates the integration of computer vision, inverse kinematics, and robot control. Using a camera and ArUco markers, the system detects marker positions, transforms them to the robot's coordinate frame, computes joint angles using inverse kinematics, and sends commands to the robot for precise movement.

----------

## Features

-   **Camera Calibration:** Live calibration using ArUco GridBoard to obtain accurate camera intrinsic parameters.
-   **Marker Detection:** Detects specific ArUco markers and estimates their poses.
-   **Robot Kinematics:** Uses `ikpy` to model the robot's kinematic chain and solve inverse kinematics for target positions.
-   **Position Transformation:** Transforms detected marker positions from the camera frame to the robot's reference frame.
-   **Robot Control:** Sends calculated joint angles to the robot for execution (currently set up for MyCobot).

----------

## Setup and Installation

### Clone the Repository

    git clone <repository_url>
    cd <repository_name>
 

### Virtual Environment and Dependencies

Create a virtual environment and install required packages.

1.  **Create a virtual environment:**

 *It is recommended to work inside a virtual environment.*
    
    python -m venv aruco_venv
    source venv/bin/activate (Windows)
    venv\Scripts\activate (Mac)
    
2.  **Install dependencies:**
 
    
    `pip install -r requirements.txt` 
    

----------

## Usage Instructions

### Camera Calibration

Run the following command to calibrate the camera using a live ArUco GridBoard setup:

`python main.py` 

The calibration process captures frames, detects ArUco markers, and computes the camera matrix and distortion coefficients. These parameters are used for accurate pose estimation.

### Main Workflow

1.  **Detect ArUco Markers:** Detect two predefined ArUco markers and estimate their poses.
2.  **Transform Coordinates:** Map detected marker coordinates from the camera frame to the robot's reference frame.
3.  **Compute Joint Angles:** Use inverse kinematics to calculate the robot's joint angles for precise end-effector positioning.
4.  **Send Commands (Optional):** Uncomment the TCP socket communication section to send movement commands to the robot.

----------

## Code Overview

### 1. `calibrate_camera_live()`

-   Captures live video frames for ArUco marker detection.
-   Calibrates the camera to calculate intrinsic parameters, distortion coefficients, and transformation matrices.

### 2. `detect_aruco_markers_and_poses()`

-   Detects ArUco markers using the calibrated camera.
-   Estimates the translation and rotation vectors of the specified markers.

### 3. `transform_to_robot_frame(tvec_camera, transformation_matrix)`

-   Transforms marker positions from the camera's coordinate frame to the robot's reference frame using a predefined transformation matrix.

### 4. `setup_robot_kinematics()`

-   Defines the robot's kinematic chain using the `ikpy` library based on the robot's specifications.

### 5. `calculate_joint_angles(robot_chain, target_position)`

-   Solves the inverse kinematics problem for the given target position and computes the joint angles.

### 6. `send_tcp_packet(client_socket, message)`

-   Sends movement commands to the robot using a TCP socket connection.

----------

## Hardware and Software Requirements

### Hardware

-   A camera (e.g., webcam or USB camera).
-   ArUco markers and a GridBoard.
-   A 6-DOF robot arm (e.g., MyCobot Pro).
-   A computer for running the Python scripts.

### Software

-   Python 3.7+
-   OpenCV (4.10.0)
-   NumPy
-   ikpy (Inverse kinematics library)

----------

## Notes

-   Ensure that the camera is positioned and calibrated properly for accurate marker detection.
-   Verify the transformation matrix (`transformation_matrix`) between the camera and robot frames using lab measurements.
-   Uncomment and customize the TCP communication section in the main script to interface with your robot.
-   The `requirements.txt` file should include all necessary dependencies like `opencv-python-contrib`, `ikpy`, and `numpy`.
- **Do not** install `opencv-python` and `opencv-python-contrib`. This will cause issues. Just install `opencv-python-contrib`

----------

## Example Output

### Detected Marker Poses


    Marker 1 Pose (Camera Frame):
    Translation Vector: [-0.5, -0.7, 0.8]
    Rotation Vector: [0.1, 0.2, 0.3]

 

### Calculated Joint Angles

    Movement angles (radians) for marker 1: [0.33, 0.31, -0.14, 0.01, 0.005, 0, 0]
    Movement angles (radians) for marker 2: [0.35, 0.32, -0.12, 0.02, 0.006, 0, 0]

----------

## License

This project is licensed under the [MIT License](LICENSE).