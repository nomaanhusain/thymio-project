import rclpy
from rclpy.node import Node
import subprocess
import cv2
import numpy as np
from std_msgs.msg import String
import socket

class PublishWindDirection(Node):
    def __init__(self):
        super().__init__('publish_wind_direction')
        self.physical_ip = self.get_physical_ip()
        self.id= self.physical_ip.split('.')[-1]
        print(f"Physical Ip={self.physical_ip}, id={self.id}")
        print("CV2 Ver. = ", cv2.__version__)
        self.lens_position = 6
        self.publisher_ = self.create_publisher(String, 'wind_direction', 10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        angle_dir = self.process_image_get_direction()
        print(f"Id:{self.id}, Angle:{angle_dir}")
        msg.data = f"{self.id}:{angle_dir}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


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
        print(R)

        # Extract yaw angle (rotation around Z-axis)
        yaw = np.arctan2(R[1, 0], R[0, 0])  # R[1,0] = sin(theta), R[0,0] = cos(theta)
        
        # Convert to degrees
        yaw_degrees = np.degrees(yaw)
        
        return yaw_degrees
    
    def process_image_get_direction(self, img_name='cam_pic.jpg'):
        self.take_picture(img_name)
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
        cv2.imwrite(f'processed_{img_name}', image)
        return angle_degrees
        
    def take_picture(self, save_image_name):
        capture_dir = "captures"
        # image_class = 'random'
        
        tuning_file = "af_jsons/ov5647_af.json"
        # image_path = os.path.join(capture_dir, image_class, f"latest.jpg")
        # print(image_path)
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
            subprocess.run(command, check=True, stdout = subprocess.DEVNULL, stderr = subprocess.STDOUT) #stdout, stderr supress terminal output
            # print(f"Image saved at: {image_path}")
        except subprocess.CalledProcessError as e:
            print(f"Error capturing image: {e}")

def main(args=None):
    rclpy.init(args=args)

    wind_direction_sensor = PublishWindDirection()

    rclpy.spin(wind_direction_sensor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wind_direction_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
