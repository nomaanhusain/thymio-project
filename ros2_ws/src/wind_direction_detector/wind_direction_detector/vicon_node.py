import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vicon_receiver.msg import PositionList
import json
from service_messages.srv import GetPoseData
import time
from std_msgs.msg import String
import numpy as np
import datetime
import math
import socket

class ViconNode(Node):
    def __init__(self):
        super().__init__('vicon_node')
        # Get your id
        self.physical_ip = self.get_physical_ip()
        self.id= self.physical_ip.split('.')[-1]
        self.id=f'id{self.id}'
        self.get_logger().info(f"Physical Ip={self.physical_ip}, id={self.id}")
        with open("wifi_id_mappings.json", "r") as f:
            wifi_id_mappings = json.load(f)
        self.id=wifi_id_mappings.get(self.id)

        self.get_logger().info(f"Updated ID from mapping: {self.id}")

        # Subscribe to the Vicon topic
        vicon_topic = "vicon/default/data"
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vicon_subscription =  self.create_subscription(
            PositionList,
            vicon_topic,
            self.vicon_callback,
            qos_profile
        )
        self.get_logger().info(f"Subscribed to: {vicon_topic}")

        # All variables for neighbours, the intention is to send back a list with the ids of all the neighbours
        self.my_position_xy = [0.0, 0.0]
        self.my_position = None
        self.neighbours_by_id = set()
        self.all_robots_position_by_id_vicon = dict()
        self.global_frame_number = 0
        self.neighbourhood_size = 3
        self.neighbour_distance_mm = 900
        self.start_time_ = time.time()

        self.node_start_time = time.time()
        self.debug_timer = time.time()
        self.runtime = 1200

        # The yaw of the robot in global reference frame
        with open("vicon_angle_correction.json", "r") as f:
                self.corrections = json.load(f)

        self.my_vicon_yaw = 0.0

        self.srv = self.create_service(
            GetPoseData,
            f'get_pose_data/{self.id}',
            self.handle_get_pose_data
        )

    def handle_get_pose_data(self, request, response):
        response.my_position_xy = self.my_position_xy
        response.my_vicon_yaw   = self.my_vicon_yaw
        response.neighbours_by_id = list(self.neighbours_by_id)
        response.global_frame_number = self.global_frame_number

        self.get_logger().info(
            f"Request received. Responding with pos={response.my_position_xy}, "
            f"yaw={response.my_vicon_yaw}, neigh={response.neighbours_by_id}"
            f"Response to be returned: {response}"
        )
        return response

    def vicon_callback(self, msg):

        if time.time() - self.start_time_ > 1: #vicon message every X secs, changed form 3 secs to 1 sec

            self.start_time_ = time.time()
            for i in range(msg.n):
                # you are expected to set the id of each robot in the vicon system, you will receive it and compare here
                if msg.positions[i].subject_name == self.id:
                    self.my_position = msg.positions[i]
                    self.my_position_xy = [msg.positions[i].x_trans, msg.positions[i].y_trans]
                    vicon_yaw_radians = self.quaternion_to_yaw()
                    self.global_frame_number = msg.positions[i].frame_number

                    self.my_vicon_yaw = vicon_yaw_radians * (180/np.pi) # rads to degrees

                    self.get_logger().info(f'New Vicon Message= {self.my_position_xy}, yaw: {self.my_vicon_yaw}, frame_no: {self.global_frame_number}')

                else:
                    self.all_robots_position_by_id_vicon[msg.positions[i].subject_name] = [msg.positions[i].x_trans, msg.positions[i].y_trans]
            self.neighbours_by_id.clear() # clear to keep updated the neighbours and remove old ones
            for id_vicon in self.all_robots_position_by_id_vicon:
                if self.all_robots_position_by_id_vicon[id_vicon] == [0,0]: continue
                if self.check_distance(self.my_position_xy, self.all_robots_position_by_id_vicon[id_vicon]):
                    self.neighbours_by_id.add(id_vicon)
            if time.time() - self.debug_timer > 10:
                self.get_logger().info(f'Vicon Position= {self.my_position_xy}, yaw: {self.my_vicon_yaw}')
                # self.get_logger().info(f"Neighbourhood size: {len(self.neighbours_by_id)}")
                self.get_logger().info(f"Neighbours: {self.neighbours_by_id}")
                self.debug_timer = time.time()

            # now = datetime.datetime.now()
            # current_time = now.strftime("%H:%M:%S")
            # self.get_logger().info(f"TIME: Vicon pose update at: {current_time}")
            if time.time() - self.node_start_time > self.runtime:
                self.get_logger().info("Vicon node, runtime finished, raising exception")
                raise Exception



    def quaternion_to_yaw(self):
        """Convert a quaternion (x, y, z, w) to a yaw angle (in radians)."""
        w = round(self.my_position.w, 4)
        x_rot = round(self.my_position.x_rot,4)
        y_rot = round(self.my_position.y_rot,4)
        z_rot = round(self.my_position.z_rot, 4)
        # self.get_logger().info(f"w={w} x_rot={x_rot}  y_rot={y_rot} z_rot={z_rot}")
        yaw = np.arctan2(2 * (w * z_rot + x_rot * y_rot), 1 - 2 * (y_rot ** 2 + z_rot ** 2))
        # yaw_degrees = np.degrees(yaw)
        # print('calculated yaw: %f' %(yaw_degrees % 360))
        return yaw + self.corrections.get(self.id)
    
    def get_physical_ip(self):
        try:
            # Connect to an external host to get the local network IP
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))  # Using Google's DNS server
                ip_address = s.getsockname()[0]
            return ip_address
        except Exception as e:
            return f"Error: {e}"
    
    def check_distance(self, my_pos, neighbour_pos):
        distance = math.sqrt((neighbour_pos[0] - my_pos[0])**2 + (neighbour_pos[1] - my_pos[1])**2)
        if distance < self.neighbour_distance_mm:
            return True
        return False

def main():

    try:

        rclpy.init()
        vicon_node = ViconNode()
        rclpy.spin(vicon_node)

    except KeyboardInterrupt:
        vicon_node.get_logger().info("Stopping, Keyboard Interrupt")
        vicon_node.destroy_node()
    except Exception:
        vicon_node.get_logger().info("Vicon Node Stopping, Times up")
        vicon_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()