from thymiodirect import Connection
from thymiodirect import Thymio
import time
import random


class RandomRobotMove():
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

    def move_forward(self):
        if self.robot is not None:
            counter = 20000
            while counter > 0:
                # if counter % 1000 == 0:
                #     print(f"Rotation={counter}")
                self.robot['motor.left.target'] = 200
                self.robot['motor.right.target'] = 200
                counter -= 1
                if self.robot['prox.horizontal'][2] > 3000:
                    self.stop()
                    self.stop_bool = True
                    break

            else:
                # print("robot stop")
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0

    def rotate_right(self):
        if self.robot is not None:
            counter = 5000
            while counter > 0:
                # if counter % 1000 == 0:
                #     print(f"Rotation={counter}")
                self.robot['motor.left.target'] = 200
                self.robot['motor.right.target'] = -200
                counter -= 1
                if self.robot['prox.horizontal'][2] > 3000:
                    self.stop()
                    self.stop_bool = True
                    break
            else:
                # print("robot stop")
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0

    def rotate_left(self):
        if self.robot is not None:
            counter = 5000
            while counter > 0:
                # if counter % 1000 == 0:
                #     print(f"Rotation={counter}")
                self.robot['motor.left.target'] = -200
                self.robot['motor.right.target'] = 200
                counter -= 1
                if self.robot['prox.horizontal'][2] > 3000:
                    self.stop()
                    self.stop_bool = True
                    break
            else:
                # print("robot stop")
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0

    def move_back(self):
        if self.robot is not None:
            counter = 20000
            while counter > 0:
                # if counter % 1000 == 0:
                #     print(f"Rotation={counter}")
                self.robot['motor.left.target'] = -200
                self.robot['motor.right.target'] = -200
                counter -= 1
                if self.robot['prox.horizontal'][2] > 3000:
                    self.stop()
                    self.stop_bool = True
                    break
            else:
                # print("robot stop")
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0

    def stop(self):
        if self.robot is not None:
            self.robot['motor.left.target'] = 0
            self.robot['motor.right.target'] = 0

    def random_move(self):
        while True:
            if self.stop_bool:
                print("Stopping Robot")
                self.stop()
                break
            self.move_forward()
            if bool(random.getrandbits(1)):
                self.rotate_right()
            else:
                self.rotate_left()
        self.stop_bool = False


