
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from thymiodirect import Connection
from thymiodirect import Thymio
import time
import os
import glob
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList

class TemperatureSensor:
    def __init__(self):
        os.system('modprobe w1-gpio')
        os.system('modprobe w1-therm')

        base_dir = '/sys/bus/w1/devices/'
        device_folder = glob.glob(base_dir + '28*')[0]
        self.device_file = device_folder + '/w1_slave'
        

    def __read_temp_raw(self):
        f = open(self.device_file, 'r')
        lines = f.readlines()
        f.close()
        return lines

    def get_temp_c(self):
        print("Getting temperature")
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

class Robot:
    def __init__(self):
        port = Connection.serial_default_port()
        th = Thymio(serial_port=port, on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))
        # Connect to Robot
        th.connect()
        self.robot = th[th.first_node()]
        # Delay to allow robot initialization of all variables
        time.sleep(1)

    def move_forward(self):
        self.robot['motor.left.target'] = 100
        self.robot['motor.right.target'] = 100

    def stop(self):
        self.robot['motor.left.target'] = 0
        self.robot['motor.right.target'] = 0

    def prox_front_active(self):
        if self.robot['prox.horizontal'][0] > 2000 or self.robot['prox.horizontal'][4] > 2000:
            return True
        return False

    def prox_rear_active(self):
        if self.robot['prox.horizontal'][5] > 2000 or self.robot['prox.horizontal'][6] > 2000:
            return True
        return False

    def move_backward(self):
        self.robot['motor.left.target'] = -100
        self.robot['motor.right.target'] = -100

class MinimalPublisher(Node):


    def __init__(self):
        super().__init__('minimal_publisher')

        self.robot_control = Robot()
        self.temp_sensor = TemperatureSensor()
        self.publisher_ = self.create_publisher(String, 'temp_topic', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.main_callback)
        self.i = 0
        self.posList: PositionList = None


    def main_callback(self):
        if self.robot_control.prox_front_active():
            print("Prox Front act")
            self.robot_control.move_backward()
        elif self.robot_control.prox_rear_active():
            print("Prox Rear act")
            self.robot_control.move_forward()
        else:
            self.robot_control.stop()
        msg = String()
        msg.data = f"Temp: {self.temp_sensor.get_temp_c()} C"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)


    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
