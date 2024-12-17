import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import os
import glob
import time
class SubscribeQuadrant(Node):
    def __init__(self):
        super().__init__('subscribe_quadrant')


        #Init temp read
        os.system('modprobe w1-gpio')
        os.system('modprobe w1-therm')
        base_dir = '/sys/bus/w1/devices/'
        device_folder = glob.glob(base_dir + '28*')[0]
        self.device_file = device_folder + '/w1_slave'

        pub_sub_topic = 'quadrant_topic'
        # I will format the string "Id:1 Q:3"
        self.subscription = self.create_subscription(
            String,
            pub_sub_topic,
            self.quad_listener_callback,
            10
        )
        self.publisher = self.create_publisher(String,pub_sub_topic,10)
        self.timer = self.create_timer(0.1,self.publish_quadrant)
        print(f"Subscribed to {pub_sub_topic}")
        self.subscription

        self.received_quadrants = dict()
        self.personal_info_weight = 0.6
        self.my_quadrant_opinion = random.randint(1,4)
        self.my_quadrant_from_sensor = self.get_quad_from_temp()
        self.social_info_counts = dict()
        self.neighbours_opinions = dict()
        self.informed = True
        
    def publish_quadrant(self):
        msg = String()
        msg.data = f"Id:{self.id} Q:{self.my_quadrant_opinion}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        
    def get_quad_from_temp(self):
        temp_c = self.get_temp_c()
        if temp_c >= 25 and temp_c < 30:
            return 1
        if temp_c >= 30 and temp_c < 45:
            return 2
        if temp_c >= 45 and temp_c < 50:
            return 3
        return 4
    
    def majority_vote(self):
        self.social_info_counts.clear()

        for quad_vals in self.neighbours_opinions.values():
            if quad_vals in self.social_info_counts:
                self.social_info_counts[quad_vals] += 1
            else:
                self.social_info_counts[quad_vals] = 1

    def make_quad_opinion(self):
        m = len(self.neighbours_opinions)
        decision_dict = dict()
        for quad in [1,2,3,4]:
            if self.informed:
                mi = self.social_info_counts.get(quad,0)

                decision_dict[quad] = mi + (self.personal_info_weight
                                            * m * 
                                            (1 if quad == self.my_quadrant_from_sensor else 0))
                
        if self.informed:
            self.my_quadrant_opinion = self.get_highest_in_dict(decision_dict)
        else:
            self.my_quadrant_opinion = self.get_highest_in_dict(self.social_info_counts)

    def get_temp_c(self):
        f = open(self.device_file, 'r')
        lines = f.readlines()
        f.close()
        lines = self.__read_temp_raw()
        while lines[0].strip()[-3:] != 'YES':
            time.sleep(0.2)
            lines = self.__read_temp_raw()
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos + 2:]
            temp_c = float(temp_string) / 1000.0
            return temp_c
        return None
    
    def quad_listener_callback(self,msg):
        split_msg = msg.split()
        id_part = int(split_msg[0].split(":")[1])  # Extract and convert Id to integer
        q_part = int(split_msg[1].split(":")[1])   # Extract and convert Q to integer
        self.neighbours_opinions[id_part] = q_part
        self.get_logger().info(f"msg: {msg.data}")
        # Sound logic, decision making would be triggered only after receiving of some message, so record temp after receiving message
        self.my_quadrant_from_sensor = self.get_quad_from_temp()

    def get_highest_in_dict(self, the_dict):
        max_count = max(the_dict.values())
        keys_with_max_count = [key for key, value in the_dict.items() if value == max_count]

        if len(keys_with_max_count) > 1:
            return random.choice(keys_with_max_count)
        else:
            return keys_with_max_count[0]
    
    
    

