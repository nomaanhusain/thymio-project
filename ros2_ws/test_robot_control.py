
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
        self.consecutive_ir_hits = 0
        self.forward_speed = 100
        self.stopped = True


    def move_forward(self):
        if self.robot is not None:
            move_time = 30
            start_time= time.time()
            # counter = 25000
            while (time.time() - start_time) < move_time:
                self.stopped = False
                if self.check_stop_all_motion():
                    print("check_stop: move forward")
                    self.stop_bool = True
                    break
                self.robot['motor.left.target'] = self.forward_speed
                self.robot['motor.right.target'] = self.forward_speed
                self.forward_speed += 10
                if self.on_arena_edge():
                    print('on arena edge')
                    self.collision_avoidance()
                    start_time = time.time()
                    continue
                if self.obstacle_ahead():
                    print('collision avoidance')
                    self.collision_avoidance()
                    start_time = time.time()
                    continue
                # counter -= 1

            else:
                self.stopped = True
                self.forward_speed = 100
                self.robot['motor.left.target'] = 0
                self.robot['motor.right.target'] = 0
    


    def on_arena_edge(self):
        if self.robot is not None:
            # print(f"Prox 0: {self.robot['prox.ground.delta'][0]}, Prox 1: {self.robot['prox.ground.delta'][1]}")
            if self.robot['prox.ground.delta'][0] > 800  or self.robot['prox.ground.delta'][1] > 800:
                return True
        return False
    

    def stop(self):
        if self.robot is not None:
            self.robot['motor.left.target'] = 0
            self.robot['motor.right.target'] = 0
    
    def collision_avoidance(self):
        if self.robot is not None:
            print("Starting Collision avoidance")
            self.robot['leds.top'] = [32, 0, 0]
            counter = 5000
            print("Move back")
            while counter > 0:
                if self.robot['prox.horizontal'][5] > 600 or self.robot['prox.horizontal'][6] > 600:
                    self.stop()
                    continue
                self.robot['motor.left.target'] = -200
                self.robot['motor.right.target'] = -200
                counter -= 1
            counter = 5000
            print("Turn")
            while counter > 0:
                self.robot['motor.left.target'] = -350
                self.robot['motor.right.target'] = 350
                counter -= 1
            self.robot['leds.top'] = [0, 32, 0]

    def obstacle_ahead(self):
        if self.robot is not None:
            readings = self.robot['prox.horizontal'][:5]
            # print(readings)
            if any(r > 800 for r in readings):
                self.consecutive_ir_hits += 1
                # print(f"IR hits: {self.consecutive_ir_hits}")
                # Temporal filtering as IR strobes from Vicon system trigger collision avoidance
                if self.consecutive_ir_hits > 600:
                    print("consecutive hits: ", self.consecutive_ir_hits)
                    print(f"Prox 0: {self.robot['prox.horizontal'][0]}, Prox 1: {self.robot['prox.horizontal'][1]},  Prox 2: {self.robot['prox.horizontal'][2]}, Prox 3: {self.robot['prox.horizontal'][3]}")
                    return True
            else:
                if self.consecutive_ir_hits > 0:
                    pass
                    # print("consecutive hits: ", self.consecutive_ir_hits)
                self.consecutive_ir_hits = 0
        return False
    def check_stop_all_motion(self):
        if self.robot is not None:
            # print(f"Prox 0: {self.robot['prox.ground.delta'][0]}, Prox 1: {self.robot['prox.ground.delta'][1]}")
            if self.robot['prox.ground.delta'][0] < 10 or self.robot['prox.ground.delta'][1] < 10:
                print("Robot lifted")
                return True
        return False
    
rob = TestRobotControl()
try:
    while rob.stopped:
        rob.move_forward()
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    rob.stop()