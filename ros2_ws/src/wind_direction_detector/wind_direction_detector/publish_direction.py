import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import subprocess
import cv2
import numpy as np
from std_msgs.msg import String
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList
from service_messages.srv import GetPoseData
from service_messages.srv import GetOpinionData
from scipy.spatial.transform import Rotation as R
import math
import socket
import time
import datetime
import csv
import json

class PublishWindDirection(Node):
    def __init__(self):
        super().__init__('publish_wind_direction')
        self.physical_ip = self.get_physical_ip()
        self.id= self.physical_ip.split('.')[-1]
        self.id=f'id{self.id}'
        self.get_logger().info(f"Physical Ip={self.physical_ip}, id={self.id}")
        with open("wifi_id_mappings.json", "r") as f:
            wifi_id_mappings = json.load(f)
        self.id=wifi_id_mappings.get(self.id)

        self.get_logger().info(f"Updated ID from mapping: {self.id}")
        
        print("CV2 Ver. = ", cv2.__version__)
        self.lens_position = 8
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(String, f'wind_direction/{self.id}', qos_profile)
        self.get_logger().info(f"Publsihing on: wind_direction/{self.id}")

        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.make_wind_direction_opinion)

        self.informed = True
        self.social_info_counts = dict()
        self.personal_info_weight = 0.6

        with open("uninformed_mapping.json", "r") as f:
            uniformed_mapping = json.load(f)
        
        if(uniformed_mapping.get(self.id)==0): self.personal_info_weight=0.0


        self.my_wind_direction = np.random.choice(['N','S','E','W'])
        self.cam_angle = -999.0
        # Random wind direction init
        self.my_wind_direction_opinion = np.random.choice(['N','S','E','W'])
        self.node_start_time = time.time()
        self.csv_file = f"log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_index = 0
        self.runtime = 1200
        self.robot_yaw = 0.0
        self.neighbours_by_id = set()
        self.global_frm_number = 0

        ## Setup the requests to the services
        self.pose_cli = self.create_client(GetPoseData, f'get_pose_data/{self.id}')
        self.opinions_cli = self.create_client(GetOpinionData, f'get_neighbours_opinion/{self.id}')

        if not self.pose_cli.wait_for_service(timeout_sec=25.0):
            self.get_logger().error('get_pose_data service not available!')
        if not self.opinions_cli.wait_for_service(timeout_sec=25.0):
            self.get_logger().error('get_opinions service not available!')

        self.init_csv()

    def init_csv(self):
        # Always overwrite the file at the start of the node
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 'timestamp', 'global_frame_number', 'message', 'cam_wind_direction'])

    def on_pose_response(self, fut):
        try:
            #get cam angle
            self.cam_angle = self.process_image_get_direction()
            resp = fut.result()
            self.get_logger().info(
                f"Pose Response: pos={resp.my_position_xy} yaw={resp.my_vicon_yaw} frame_number = {resp.global_frame_number}")
            self.robot_yaw = resp.my_vicon_yaw
            self.neighbours_by_id = resp.neighbours_by_id
            self.global_frm_number = resp.global_frame_number
            #after you have the pose info, find true wind direction
            self.get_true_wind_direction(self.cam_angle)
            # Neighbour Opionions request
            self.get_logger().info("Neighbour info requested")
            n_op_req = GetOpinionData.Request()
            n_op_req.neighbours_by_id = self.neighbours_by_id
            n_op_future = self.opinions_cli.call_async(n_op_req)
            n_op_future.add_done_callback(self.get_opinion_make_decision)

        except Exception as e:
            self.get_logger().error(f"[func:on_pose_response] Service call failed: {e}")

        
    def get_opinion_make_decision(self, op_fut):
        try:
            op_resp = op_fut.result()
            self.get_logger().info(f"Neighbour Opinion Response: {op_resp}")
            neighbours_opinions = op_resp.neighbours_opinions
            self.majority_vote(neighbours_opinions)
            m = len(neighbours_opinions)
            self.get_logger().info(f"Inside Decision Making, neighbours opinion: {neighbours_opinions}")
            self.get_logger().info(f"Inside Decision Making, computed social info: {self.social_info_counts}")
            decision_dict = dict()
            self.get_logger().info(f"Inside Decision Making, My Wind Direction: {self.my_wind_direction}")
            for direction in ['N','S','E','W']:
                if self.informed:
                    mi = self.social_info_counts.get(direction,0)

                    decision_dict[direction] = mi + (self.personal_info_weight
                                                * m * 
                                                (1 if direction == self.my_wind_direction else 0))

            
            if self.informed:
                self.get_logger().info(f"Inside Decision Making, Decision dict: {decision_dict}")
                self.my_wind_direction_opinion = self.get_highest_in_dict(decision_dict)
            else:
                self.get_logger().info(f"Uninformed decision making: {self.social_info_counts}")
                self.my_wind_direction_opinion = self.get_highest_in_dict(self.social_info_counts)
            
            timestamp = datetime.datetime.now().isoformat()
            self.csv_index += 1
            # Log to CSV
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([self.csv_index, timestamp, self.global_frm_number, self.my_wind_direction_opinion, self.my_wind_direction])
                if time.time() - self.node_start_time > 600 and time.time() - self.node_start_time < 606:
                    writer.writerow(['*','*','*','*','*'])

            # Publishing stuff to the wind_direction topic
            msg = String()

            msg.data = f"{self.id}:{self.my_wind_direction_opinion}"
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')
            now = datetime.datetime.now()
            current_time = now.strftime("%H:%M:%S")
            self.get_logger().info(f"TIME: Decision making completed at: {current_time}.")
            self.get_logger().info(f"-----------------------------------------------------")

        except Exception as e:
            self.get_logger().error(f"[func:get_opinion_make_decision] Service call failed: {e}")

        

    

    def make_wind_direction_opinion(self):
        # Pose Request
        self.get_logger().info("Main Pub Node: Requesting Pose Info")
        pose_req = GetPoseData.Request()
        pose_future = self.pose_cli.call_async(pose_req)
        pose_future.add_done_callback(self.on_pose_response)

        if time.time() - self.node_start_time > self.runtime:
            now = datetime.datetime.now()
            current_time = now.strftime("%H:%M:%S")
            self.get_logger().info(f'Raising exception at {current_time}')
            raise Exception

    def get_highest_in_dict(self, the_dict):
        max_count = max(the_dict.values())
        keys_with_max_count = [key for key, value in the_dict.items() if value == max_count]

        if len(keys_with_max_count) > 1:
            return np.random.choice(keys_with_max_count)
        else:
            return keys_with_max_count[0]
        
    
    
    def get_true_wind_direction(self, cam_angle):
        ground_truth_wind_direction = (cam_angle + self.robot_yaw) % 360
        self.get_logger().info(f"***Cam angle= {cam_angle}, vico ang deg= {self.robot_yaw} Ground Truth= {ground_truth_wind_direction}")
        self.my_wind_direction = '1'
        if (ground_truth_wind_direction < 45 and ground_truth_wind_direction >= 0) or (ground_truth_wind_direction < 360 and ground_truth_wind_direction >= 315):
            self.my_wind_direction = 'N'
        if ground_truth_wind_direction >= 225 and ground_truth_wind_direction < 315:
            self.my_wind_direction = 'E'
        if ground_truth_wind_direction >= 135 and ground_truth_wind_direction < 225:
            self.my_wind_direction = 'S'
        if ground_truth_wind_direction >= 45  and ground_truth_wind_direction < 135:
            self.my_wind_direction = 'W'
        if self.my_wind_direction == '1':
            self.my_wind_direction = np.random.choice(['N','S','E','W'])
        # self.get_logger().info(f"***Corrected Wind Direction= {self.my_wind_direction}")
        

    def majority_vote(self, n_opinions):
        self.social_info_counts.clear()
        for wind_dir in n_opinions:
            if wind_dir in self.social_info_counts:
                self.social_info_counts[wind_dir] += 1
            else:
                self.social_info_counts[wind_dir] = 1
        


    def get_physical_ip(self):
        try:
            # Connect to an external host to get the local network IP
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))  # Using Google's DNS server
                ip_address = s.getsockname()[0]
            return ip_address
        except Exception as e:
            return f"Error: {e}"
    
    def my_estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
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
    
    def process_image_get_direction(self, img_name='cam_pic.jpg'):
        self.take_picture(img_name)
        time.sleep(0.5) #originally 2 sec
        print("--Processing Picture--")
        # Load the image
        image = cv2.imread(f'{img_name}')

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
        else:
            print("No ArUco marker detected.")

        camera_calib_data = np.load('camera_calibration_data.npz')
        camera_matrix = camera_calib_data['camera_matrix']
        dist_coeffs = camera_calib_data['dist_coeffs']

        marker_length = 0.02375 #in meters, it is 23.75mm
        angle_degrees = -999.0
        true_cam_angle = -999.0
        if ids is not None:
            rvec, tvec, _ = self.my_estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            for idx in range(len(ids)):
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec[idx], tvec[idx], 0.05)
                angle_degrees= self.get_marker_rotation_angle(rvec[idx])

                #angle_degrees is at a 90 degree offset to robot front as the cam is mounted at 90 degrees, also the angles range from 0-180 and -1 to -180
                true_cam_angle = angle_degrees - 90
                # if true_cam_angle < 0 : true_cam_angle = true_cam_angle + 360
                # print("Angle = ",angle_degrees)
        image = cv2.putText(image, f'Angle={true_cam_angle:.2f}', (1296,972), cv2.FONT_HERSHEY_SIMPLEX, 
                   fontScale=2, color=(5, 10, 255), thickness=5, lineType=cv2.LINE_AA)
        cv2.imwrite(f'processed_{img_name}', image)
        print("--Picture Processing Done--")
        return true_cam_angle
        
    def take_picture(self, save_image_name):
        print("--Taking picture--")
        
        tuning_file = "af_jsons/ov5647_af.json"
        # Run the libcamera-still command with autofocus configuration
        command = [
            "libcamera-still",
            "-o", save_image_name,
            "--immediate",
            "--autofocus-mode", "manual",
            "--lens-position", str(self.lens_position),
            "--tuning-file", tuning_file
        ]

        try:
            subprocess.run(command, check=True, stdout = subprocess.DEVNULL, stderr = subprocess.DEVNULL, timeout=5)
        except subprocess.TimeoutExpired:
            print("--Timeout: libcamera-still took too long. Skipping this attempt.")
        except subprocess.CalledProcessError as e:
            print(f"Error capturing image: {e}")

def main(args=None):

    try:
        rclpy.init(args=args)

        wind_direction_sensor = PublishWindDirection()

        rclpy.spin(wind_direction_sensor)

    except KeyboardInterrupt:
        wind_direction_sensor.get_logger().info("Stopping, Keyboard Interrupt")
        wind_direction_sensor.destroy_node()
    # except Exception:
    #     wind_direction_sensor.get_logger().info("Publisher stopping, exception")
    #     wind_direction_sensor.destroy_node()
    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()
