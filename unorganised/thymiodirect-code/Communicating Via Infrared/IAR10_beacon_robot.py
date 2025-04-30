# imports
import argparse
import time
import math
import random
from pathlib import Path 
# import thymiodirect package
from thymiodirect import Connection 
from thymiodirect import Thymio

def main(use_sim=False, ip='localhost', port=2001):
    ''' Main function '''
    
    # assembler programs to en-/disable infrared communication
    __PCE_ASM = Path("prox_comm_enable.asm")
    __PCD_ASM = "prox_comm_disable.asm"

    try:
        # Configure Interface to Thymio robot
        # simulation
        if use_sim:
            th = Thymio(use_tcp=True, host=ip, tcp_port=port, 
                        on_connect=lambda node_id: print(f' Thymio {node_id} is connected'))
        # real robot
        else:
            port = Connection.serial_default_port()
            th = Thymio(serial_port=port, 
                        on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))

        # Connect to Robot
        th.connect()
        robot = th[th.first_node()]

        # Delay to allow robot initialization of all variables
        time.sleep(1)
        
        # enable infrared communication
        asm = None
        with __PCE_ASM.open("r") as file:
            asm = file.read()
        id = th.first_node()
        th.run_asm(id, asm)
        th[id]["prox.comm.tx"] = int(30) # set temperature value to be send via infrared LEDs
        
        # BEECLUST
        state = "find"
        ts = time.time()
        direction = random.choice(["left", "right"])
        
        # Main loop
        while True:
            # send?
            th[id]["prox.comm.tx"] = int(30)
            #print(th[id]["prox.comm.rx"])
             
                        
    except Exception as err:
        # Stop robot
        robot['motor.left.target'] = 0
        robot['motor.right.target'] = 0 
        print(err)


if __name__ == '__main__':
    # Parse commandline arguments to cofigure the interface for a simulation (default = real robot)
    parser = argparse.ArgumentParser(description='Configure optional arguments to run the code with simulated Thymio. '
                                                    'If no arguments are given, the code will run with a real Thymio.')
    
    # Add optional arguments
    parser.add_argument('-s', '--sim', action='store_true', help='set this flag to use simulation')
    parser.add_argument('-i', '--ip', help='set the TCP host ip for simulation. default=localhost', default='localhost')
    parser.add_argument('-p', '--port', type=int, help='set the TCP port for simulation. default=2001', default=2001)

    # Parse arguments and pass them to main function
    args = parser.parse_args()
    main(args.sim, args.ip, args.port)
