
from thymiodirect import Connection
from thymiodirect import Thymio

import time

class TestRobotControl:
    def __init__(self):
        print("Initializing Thymio Robot")
        port = Connection.serial_default_port()
        th = Thymio(serial_port=port,
                    on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))
        # Connect to Robot
        th.connect()
        self.robot = th[th.first_node()]
        time.sleep(2)
        # b) print all variables
        print(th.variables(th.first_node()))
        print("Robot connected")
        self.forward_speed = 100


    def move_forward(self):
        if self.forward_speed >= 1000: self.forward_speed = 100
        # if self.on_arena_edge:
        #     print("on edge")
        #     self.forward_speed = 0
        self.robot['motor.left.target'] = self.forward_speed
        self.robot['motor.right.target'] = self.forward_speed
        self.forward_speed +=10
    


    def on_arena_edge(self):
        print(f"Prox 0: {self.robot['prox.ground.delta'][0]}, Prox 1: {self.robot['prox.ground.delta'][1]}")
        if self.robot['prox.ground.delta'][0] > 800  or self.robot['prox.ground.delta'][1] > 800:
            return True
        return False
    

    def stop(self):
        if self.robot is not None:
            self.robot['motor.left.target'] = 0
            self.robot['motor.right.target'] = 0
    
rob = TestRobotControl()
try:
    while True:
        rob.move_forward()
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    rob.stop()