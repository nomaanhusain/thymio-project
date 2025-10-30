import subprocess
import cv2
import numpy as np
import time
import csv
import sys
def process_image_get_direction(img_name='cam_pic.jpg'):
    take_picture(img_name)
    time.sleep(0.5) #originally 2 sec
    # print("--Processing Picture--")
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
        rvec, tvec, _ = my_estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        for idx in range(len(ids)):
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec[idx], tvec[idx], 0.05)
            angle_degrees= get_marker_rotation_angle(rvec[idx])

            #angle_degrees is at a 90 degree offset to robot front as the cam is mounted at 90 degrees, also the angles range from 0-180 and -1 to -180
            true_cam_angle = angle_degrees - 90
            # if true_cam_angle < 0 : true_cam_angle = true_cam_angle + 360
            # print("Angle = ",angle_degrees)
    image = cv2.putText(image, f'Angle={true_cam_angle:.2f}', (1296,972), cv2.FONT_HERSHEY_SIMPLEX, 
            fontScale=2, color=(5, 10, 255), thickness=5, lineType=cv2.LINE_AA)
    cv2.imwrite(f'processed_{img_name}', image)
        # print("--Picture Processing Done--")
    return true_cam_angle

def take_picture(save_image_name):
    # print("--Taking picture--")
        
    tuning_file = "af_jsons/ov5647_af.json"
    # Run the libcamera-still command with autofocus configuration
    command = [
        "libcamera-still",
        "-o", save_image_name,
        "--immediate",
        "--autofocus-mode", "manual",
        "--lens-position", str("8"),
        "--tuning-file", tuning_file
    ]

    try:
        subprocess.run(command, check=True, stdout = subprocess.DEVNULL, stderr = subprocess.DEVNULL, timeout=5)
    except subprocess.TimeoutExpired:
        print("--Timeout: libcamera-still took too long. Skipping this attempt.")
    except subprocess.CalledProcessError as e:
        print(f"Error capturing image: {e}")

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
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

def get_marker_rotation_angle( rvec):
    # Convert rotation vector to rotation matrix, this uses the Rodrigues rotation formula
    R, _ = cv2.Rodrigues(rvec)
    # print(R)

    # Extract yaw angle (rotation around Z-axis)
    yaw = np.arctan2(R[1, 0], R[0, 0])  # R[1,0] = sin(theta), R[0,0] = cos(theta)
        
    # Convert to degrees
    yaw_degrees = np.degrees(yaw)
    return yaw_degrees

def main():
    timer = time.time()
    start_time =  time.time()
    file = open("output.csv", mode="w", newline="")
    writer = csv.writer(file)
    writer.writerow(['index', 'timestamp', 'angle'])
    index = 0
    time_limit = 610
    try:
        while True:
            if time.time() - timer > 5:
                cam_angle = process_image_get_direction()
                print(f"Fin Angle: {cam_angle}")
                timer = time.time()
                writer.writerow([index, timer, cam_angle])
                print("------")
                print(f"Time Left: {(timer - start_time) - time_limit}s")
                print(f"{index} | {timer} | {cam_angle}")
                print("------")
                index += 1
                file.flush()
            if timer -  start_time > time_limit:
                sys.exit(130)
            


    except KeyboardInterrupt:
        file.close()
        print("Keyboard Interrupt, stopping")
    except SystemExit:
        print("System Exited")

main()
