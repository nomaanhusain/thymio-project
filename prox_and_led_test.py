from thymiodirect import Connection
from thymiodirect import Thymio
import time
port = Connection.serial_default_port()
th = Thymio(serial_port=port, on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))

# Connect to Robot
th.connect()
robot = th[th.first_node()]

# Delay to allow robot initialization of all variables
time.sleep(1)

print(th.variables(th.first_node()))

while True:
    # print("Horizontal Prox\n")
    # for i, proxVal in enumerate(robot['prox.horizontal']):
    #     print(f"Prox {i}: {proxVal} ", end='')
    # print("\n")
    # print("Ground Prox\n")
    if robot['prox.ground.delta'][0] < 600 and robot['prox.ground.delta'][1] < 600:
        print("Robot lifted")
        robot['leds.top'] = [32, 0, 0]
        continue
    # for i,proxVal in enumerate(robot['prox.ground.delta']):
    #     print(f"Prox {i}: {proxVal} ",end='')
    if robot['prox.horizontal'][2] > 2000:
        robot['leds.top'] = [0, 0, 32]
    else:
        robot['leds.top'] = [0, 32, 0]

# li = [42,242,34,44,325,23]
# for i,val in enumerate(li):
#     print(i,val)