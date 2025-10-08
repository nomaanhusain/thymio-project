import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import json
from service_messages.srv import GetOpinionData
import time
from std_msgs.msg import String
import numpy as np
import datetime
import math
import socket

class NeighboursOpinionNode(Node):
    def __init__(self):
        super().__init__('neighbours_opinion_node')
        # Get your id
        self.physical_ip = self.get_physical_ip()
        self.id= self.physical_ip.split('.')[-1]
        self.id=f'id{self.id}'
        self.get_logger().info(f"Physical Ip={self.physical_ip}, id={self.id}")
        with open("wifi_id_mappings.json", "r") as f:
            wifi_id_mappings = json.load(f)
        self.id=wifi_id_mappings.get(self.id)

        self.get_logger().info(f"Updated ID from mapping: {self.id}")

        # Subscribe to the other robots
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # time.sleep(10)

        thymio_ips = ['id68','id136','id137','id142','id151','id152','id155','id189','id192','id193','id194','id195','id200','id206',
                      'id207','id223','id224','id236','id247','id1193']
        self.other_devices = [i for i in thymio_ips if i != self.id]
        for dev in self.other_devices:
            topic_name = f'wind_direction/{dev}'
            self.create_subscription(
                String,
                topic_name,
                self.neighbours_message_callback,
                qos_profile
            )
            self.get_logger().info(f'Subscribed to: {topic_name}')
        self.timer = self.create_timer(10, self.stop_execution)
        self.commnunication_noise = 0.2
        self.all_opinions = dict() # Keep track of all opinions you have received
        self.node_start_time = time.time()
        self.runtime = 1200

        #Create service
        self.srv = self.create_service(
            GetOpinionData,
            f'get_neighbours_opinion/{self.id}',
            self.handle_opinion_request
        )

    def stop_execution(self):
        if time.time() - self.node_start_time > self.runtime:
            now = datetime.datetime.now()
            current_time = now.strftime("%H:%M:%S")
            self.get_logger().info(f'Raising exception at {current_time}')
            raise Exception
        
    def handle_opinion_request(self, request, response):
        response.neighbours_opinions = [
            self.all_opinions[id_] 
            for id_ in request.neighbours_by_id 
            if id_ in self.all_opinions
        ]
        # TODO
        # self.get_logger().info(
        #     f"Requested {request.neighbours_by_id}, responding {response.neighbours_opinions}"
        # )
        return response


    def neighbours_message_callback(self, msg):
        # This is listening to the wind_direction topic, if someone publishes there opinion, we listen here

        id = msg.data.split(':')[0]
        wind_direction = msg.data.split(':')[1]
        if id == self.id: return # Return incase it is your own message
        # TODO
        # self.get_logger().info(f'* Received from {id}, wind direction {wind_direction}')
        if id != self.id:
            # Simulate communication noise
            if np.random.random() < self.commnunication_noise:
                other_opinions = [opinion for opinion in ['N','S','E','W'] if opinion!=wind_direction]
                wind_direction = np.random.choice(other_opinions)
                # self.get_logger().info(f"Comm. Noise, new received opinion: {wind_direction}")
            self.all_opinions[id] = wind_direction
        # self.get_logger().info(f"All opinions = {self.all_opinions}")
        self.get_logger().info(f"All opinions size= {len(self.all_opinions)}")
    
    def get_physical_ip(self):
        try:
            # Connect to an external host to get the local network IP
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))  # Using Google's DNS server
                ip_address = s.getsockname()[0]
            return ip_address
        except Exception as e:
            return f"Error: {e}"
        

def main():
    try:

        rclpy.init()
        neighbours_node = NeighboursOpinionNode()
        rclpy.spin(neighbours_node)

    except KeyboardInterrupt:
        neighbours_node.get_logger().info("Stopping, Keyboard Interrupt")
        neighbours_node.destroy_node()
    except Exception as e:
        neighbours_node.get_logger().error(e)
        neighbours_node.get_logger().info("Stopping, Exception")
        neighbours_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()