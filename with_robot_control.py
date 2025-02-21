import time
from thymiodirect import Connection 
from thymiodirect import Thymio
import numpy as np
import cv2
import subprocess
import os
import time

class DirectionRobot:
    def __init__(self):
        print("Initializing Thymio Robot")
        port = Connection.serial_default_port()
        th = Thymio(serial_port=port,
                    on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))
        # Connect to Robot
        th.connect()
        self.robot = th[th.first_node()]

        # Delay to allow robot initialization of all variables
        time.sleep(1)
        # b) print all variables
        print(th.variables(th.first_node()))
        print("Robot connected")
        self.stop_bool = False
        self.lens_position = 6

    def move_forward(self):
        if self.robot is not None:
            counter = 20000
            while counter > 0:
                if self.check_stop_all_motion():
                    # print("stop all motion: move forward")
                    self.stop_bool = True
                    break
                self.robot['motor.left.target'] = 200
                self.robot['motor.right.target'] = 200
                if self.on_arena_edge():
                    self.collision_avoidance()
                    continue
                if self.obstacle_ahead():
                    self.collision_avoidance()
                    continue
                counter -= 1

            else:
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0

    def rotate_right(self):
        if self.robot is not None:
            counter = 5000
            while counter > 0:
                if self.check_stop_all_motion():
                    # print("stop all motion: rotate right")
                    self.stop_bool = True
                    break
                self.robot['motor.left.target'] = 200
                self.robot['motor.right.target'] = -200
                if self.on_arena_edge():
                    self.collision_avoidance()
                    continue
                if self.obstacle_ahead():
                    self.collision_avoidance()
                    continue
                counter -= 1
            else:
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0
    
    def rotate_left(self):
        if self.robot is not None:
            counter = 5000
            while counter > 0:
                if self.check_stop_all_motion():
                    # print("stop all motion: rotate left")
                    self.stop_bool = True
                    break
                self.robot['motor.left.target'] = -200
                self.robot['motor.right.target'] = 200
                if self.obstacle_ahead():
                    self.collision_avoidance()
                    continue
                counter -= 1
            else:
                # print("robot stop")
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0
    
    def move_back(self):
        if self.robot is not None:
            counter = 20000
            while counter > 0:
                if self.check_stop_all_motion():
                    # print("stop all motion: move back")
                    self.stop_bool = True
                    break
                if self.robot['prox.horizontal'][5] > 600 or self.robot['prox.horizontal'][6] > 600:
                    self.stop()
                    continue
                self.robot['motor.left.target'] = -200
                self.robot['motor.right.target'] = -200
                counter -= 1
            else:
                # print("robot stop")
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0

    def check_stop_all_motion(self):
        if self.robot is not None:
            # print(f"Prox 0: {self.robot['prox.ground.delta'][0]}, Prox 1: {self.robot['prox.ground.delta'][1]}")
            if self.robot['prox.ground.delta'][0] < 10 or self.robot['prox.ground.delta'][1] < 10:
                print("Robot lifted")
                return True
        return False
    def stop(self):
        if self.robot is not None:
            self.robot['motor.left.target'] = 0
            self.robot['motor.right.target'] = 0

    def collision_avoidance(self):
        if self.robot is not None:
            # print("Starting Collision avoidance")
            self.robot['leds.top'] = [32, 0, 0]
            counter = 3000
            # print("Move back")
            while counter > 0:
                if self.robot['prox.horizontal'][5] > 600 or self.robot['prox.horizontal'][6] > 600:
                    self.stop()
                    continue
                self.robot['motor.left.target'] = -200
                self.robot['motor.right.target'] = -200
                counter -= 1
            counter = 2000
            # print("Turn")
            while counter > 0:
                self.robot['motor.left.target'] = -300
                self.robot['motor.right.target'] = 300
                counter -= 1
            self.robot['leds.top'] = [0, 32, 0]
    
    def on_arena_edge(self):
        if self.robot is not None:
            # print(f"Prox 0: {self.robot['prox.ground.delta'][0]}, Prox 1: {self.robot['prox.ground.delta'][1]}")
            if self.robot['prox.ground.delta'][0] > 900  or self.robot['prox.ground.delta'][1] > 900:
                return True
        return False

    def obstacle_ahead(self):
        if self.robot is not None:
            if (self.robot['prox.horizontal'][0] > 1000 or self.robot['prox.horizontal'][1] > 1000 or
                    self.robot['prox.horizontal'][2] > 1000 or self.robot['prox.horizontal'][3] > 1000 or self.robot['prox.horizontal'][4] > 1000):
                return True
        return False
    
    
    def my_estimatePoseSingleMarkers(self,corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash


    def get_marker_rotation_angle(self, rvec):
        # Convert rotation vector to rotation matrix, this uses the Rodrigues rotation formula
        R, _ = cv2.Rodrigues(rvec)
        # print(R)

        # Extract yaw angle (rotation around Z-axis)
        yaw = np.arctan2(R[1, 0], R[0, 0])  # R[1,0] = sin(theta), R[0,0] = cos(theta)
        
        # Convert to degrees
        yaw_degrees = np.degrees(yaw)
        
        return yaw_degrees
    
    def process_image_get_direction(self, img_name):
        self.take_picture(img_name)
        # Load the image
        image = cv2.imread(f'captures/{img_name}')

        # Define the dictionary
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()

        # Detect ArUco markers
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(image)
        # print(corners)

        if ids is not None:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(image, corners, ids, (0,0,225))
            (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0]
            # print(f"Image Name={img_name}: ")
            # print(topLeft, topRight, bottomLeft, bottomRight)
            # cv2.namedWindow("Detected_ArUco_Marker", cv2.WINDOW_NORMAL)
            # cv2.imshow("Detected_ArUco_Marker", image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        else:
            print("No ArUco marker detected.")

        camera_calib_data = np.load('camera_calibration_data.npz')
        camera_matrix = camera_calib_data['camera_matrix']
        dist_coeffs = camera_calib_data['dist_coeffs']

        marker_length = 0.02375 #in meters, it is 23.75mm
        angle_degrees = -999.0
        if ids is not None:
            rvec, tvec, _ = self.my_estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            for idx in range(len(ids)):
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec[idx], tvec[idx], 0.05)
                angle_degrees= self.get_marker_rotation_angle(rvec[idx])
                # print("Angle = ",angle_degrees)
        image = cv2.putText(image, f'Angle={angle_degrees:.2f}', (1296,972), cv2.FONT_HERSHEY_SIMPLEX, 
                   fontScale=2, color=(5, 10, 255), thickness=5, lineType=cv2.LINE_AA)
        cv2.imwrite(f'captures/processed_{img_name}', image)
        return angle_degrees
        
    def take_picture(self, save_image_name):
        capture_dir = "captures"
        # image_class = 'random'
        
        tuning_file = "af_jsons/ov5647_af.json"
        image_path = os.path.join(capture_dir, f"{save_image_name}")
        # image_path = os.path.join(capture_dir, image_class, f"latest.jpg")
        # print(image_path)
        # Run the libcamera-still command with autofocus configuration
        command = [
            "libcamera-still",
            "-o", image_path,
            "--immediate",
            "--autofocus-mode", "manual",
            "--lens-position", str(self.lens_position),
            "--tuning-file", tuning_file
        ]

        try:
            subprocess.run(command, check=True, stdout = subprocess.DEVNULL, stderr = subprocess.STDOUT) #stdout, stderr supress terminal output
            # print(f"Image saved at: {image_path}")
        except subprocess.CalledProcessError as e:
            print(f"Error capturing image: {e}")
        

if __name__ == '__main__':
    direction_robot = DirectionRobot()
    start_time = time.time()
    try:
        while True:
            if time.time() - start_time > 5:
                angle = direction_robot.process_image_get_direction("latest_img.jpg")
                print('Angle= ',angle)
                start_time = time.time()

            direction_robot.move_forward()
            if direction_robot.stop_bool:
                direction_robot.stop()
            direction_robot.rotate_right()
            if direction_robot.stop_bool:
                direction_robot.stop()
            
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        direction_robot.stop()
        
        # if keyboard.is_pressed('f'):
        #     direction_robot.move_forward()
        # if keyboard.is_pressed('l'):
        #     direction_robot.rotate_left()
        # if keyboard.is_pressed('r'):
        #     direction_robot.rotate_right()
        # if keyboard.is_pressed('b'):
        #     direction_robot.move_back()
        # if keyboard.is_pressed('q'):
        #     break


