import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import numpy as np
from std_msgs.msg import String
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList
import time
import datetime
import json
import socket


class CommunicationTest(Node):
    def __init__(self):
        super().__init__('communication_test')
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
        vicon_topic = "vicon/default/data"
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(String, f'wind_direction/{self.id}', qos_profile)
        self.get_logger().info(f"Publsihing on: wind_direction/{self.id}")

        self.vicon_subscription =  self.create_subscription(
            PositionList,
            vicon_topic,
            self.vicon_callback,
            qos_profile
        )

        self.get_logger().info(f"Subscribed to: {vicon_topic}")

        thymio_ips = ['id68','id136','id137','id142','id151','id152','id155','id189','id192','id193','id194','id195','id200','id206',
                      'id207','id223','id224','id236','id247','id1193']
        self.other_devices = [i for i in thymio_ips if i != self.id]
        for dev in self.other_devices:
            topic_name = f'wind_direction/{dev}'
            self.create_subscription(
                String,
                topic_name,
                self.listener_callback,
                qos_profile
            )
            self.get_logger().info(f'Subscribed to: {topic_name}')

        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.send_message)

        self.my_position_xy = None
        self.neighbours_by_id = set()
        self.all_robots_position_by_id_vicon = dict()
        self.all_opinions = dict()

        self.my_position = None
        self.my_wind_direction_opinion = np.random.choice(['N','S','E','W'])
        self.start_time_ = time.time()
        self.node_start_time = time.time()
        self.comm_test_set = set()
        self.bag_counter=0
        self.runtime = 300

    def vicon_callback(self, msg):
    
        # if time.time() - self.start_time_ > 0.1:
        if True:
            self.bag_counter += 1
            if time.time() - self.start_time_ > 1:
                self.start_time_ = time.time()
                self.get_logger().info(f"Vicon messages in last sec: {self.bag_counter}")
                self.bag_counter = 0
            for i in range(msg.n):
                # self.get_logger().info(f'Subject Name: {msg.positions[i].subject_name}')
                # you are expected to set the id of each robot in the vicon system, you will receive it and compare here
                if msg.positions[i].subject_name == self.id:
                    self.my_position = msg.positions[i]
                    self.my_position_xy = [msg.positions[i].x_trans, msg.positions[i].y_trans]
                    # self.get_logger().info(f'Vicon Position= {self.my_position_xy}')
                    

                else:
                    # self.get_logger().info(f'msg={msg}')
                    self.all_robots_position_by_id_vicon[msg.positions[i].subject_name] = [msg.positions[i].x_trans, msg.positions[i].y_trans]
            self.neighbours_by_id.clear() # clear to keep updated the neighbours and remove old ones
            for id_vicon in self.all_robots_position_by_id_vicon:
                if self.all_robots_position_by_id_vicon[id_vicon] == [0,0]: continue
                self.neighbours_by_id.add(id_vicon)
            

    def listener_callback(self, msg):
        # This is listening to the wind_direction topic, if someone publishes there opinion, we listen here
        # Once we have enough unique opinions, we trigger the decision making
        id = msg.data.split(':')[0]
        wind_direction = msg.data.split(':')[1]
        if id == self.id: return # Return incase it is your own message
        self.comm_test_set.add(id)
        
        self.get_logger().info("-------------------------------------------------------------------")
        self.get_logger().info(f'* Received from {id}, wind direction {wind_direction}')
        if id != self.id:
            self.all_opinions[id] = wind_direction
        self.get_logger().info(f"All opinions = {self.all_opinions}")
        self.get_logger().info(f"All opinions size= {len(self.all_opinions)}")
        self.get_logger().info("## last Vicon update ##")
        self.get_logger().info(f'Vicon Position= {self.my_position_xy}')
        self.get_logger().info(f"Robot swarm size: {len(self.neighbours_by_id)}")
        if len(self.comm_test_set) == len(self.all_opinions):
            self.get_logger().info(f"Comm Test")
            self.get_logger().info(f"Heard from {len(self.comm_test_set)}")
            self.comm_test_set.clear()
            now = datetime.datetime.now()
            current_time = now.strftime("%H:%M:%S")
            self.get_logger().info(f"All robot's message received at: {current_time}")
        
        now = datetime.datetime.now()
        current_time = now.strftime("%H:%M:%S")
        self.get_logger().info(f"TIME: Vicon pose update at: {current_time}")
        self.get_logger().info("-------------------------------------------------------------------")

    
    def send_message(self):
        opinion = np.random.choice(['N','S','E','W'])
        msg = String()
        msg.data = f"{self.id}:{opinion}"
        self.get_logger().info(f"Publishing message {msg}")
        self.publisher_.publish(msg)

        if time.time() - self.node_start_time > self.runtime:
            now = datetime.datetime.now()
            current_time = now.strftime("%H:%M:%S")
            self.get_logger().info(f'Raising exception at {current_time}')
            raise Exception
        
    def get_physical_ip(self):
        try:
            # Connect to an external host to get the local network IP
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                ip_address = s.getsockname()[0]
            return ip_address
        except Exception as e:
            return f"Error: {e}"


def main(args=None):

    try:
        rclpy.init(args=args)

        comm_node = CommunicationTest()

        rclpy.spin(comm_node)

    except KeyboardInterrupt:
        comm_node.get_logger().info("Stopping, Keyboard Interrupt")
        comm_node.destroy_node()
    except Exception:
        comm_node.get_logger().info("Publisher stopping, exception")
        comm_node.destroy_node()
    finally:
        rclpy.shutdown()