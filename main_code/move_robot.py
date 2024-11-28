from thymiodirect import Connection
from thymiodirect import Thymio
import time


class ControlRobot:
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

    def rotate_right(self):
        if self.robot is not None:
            counter = 60000
            while counter > 0:
                # if counter % 1000 == 0:
                #     print(f"Rotation={counter}")
                self.robot['motor.left.target'] = 200
                self.robot['motor.right.target'] = -200
                counter -= 1
            else:
                # print("robot stop")
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0

    def move_forward(self):
        if self.robot is not None:
            counter = 60000
            while counter > 0:
                # if counter % 1000 == 0:
                #     print(f"Rotation={counter}")
                self.robot['motor.left.target'] = 200
                self.robot['motor.right.target'] = 200
                counter -= 1
            else:
                # print("robot stop")
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0

    def move_back(self):
        if self.robot is not None:
            counter = 60000
            while counter > 0:
                # if counter % 1000 == 0:
                #     print(f"Rotation={counter}")
                self.robot['motor.left.target'] = -200
                self.robot['motor.right.target'] = -200
                counter -= 1
            else:
                # print("robot stop")
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0
