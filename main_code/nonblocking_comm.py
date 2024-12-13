import socket
import sys
import threading
import time
from thymiodirect import Connection
from thymiodirect import Thymio
import queue
import select
from random_move_robot import RandomRobotMove
from read_temp import TemperatureSensor

class PeerToPeerNode:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.peers = []  # List of peer IPs
        self.server = None
        self.running = True
        self.message_queue = queue.Queue()  # Queue for inter-thread communication

    def start_server(self):
        """Start the server to listen for incoming connections."""
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.host, self.port))
        '''server.listen()
        Enable a server to accept connections. If backlog is specified, it must be at least 0 (if it is lower, it is set to 0); 
        it specifies the number of unaccepted connections that the system will allow before refusing new connections.
        '''
        self.server.listen(5)
        print(f"Server started on {self.host}:{self.port}")
        while self.running:
            try:
                conn, addr = self.server.accept()
                print(f"Connection established with {addr}")
                threading.Thread(target=self.handle_peer, args=(conn, addr)).start()
            except Exception as e:
                print(f"Server error: {e}")
                break

    def handle_peer(self, conn, addr):
        """Handle communication with a connected peer."""
        while self.running:
            try:
                message = conn.recv(1024).decode()
                if message:
                    print(f"Message from {addr}: {message}")
                    if message == "rotate":
                        self.message_queue.put(1)
                        print(f"message_queue size = {self.message_queue.qsize()}")
                else:
                    break
            except ConnectionResetError:
                print(f"Peer {addr} disconnected")
                break
        conn.close()

    def connect_to_peers(self, peer_ips):
        """Connect to the provided peer IPs."""
        for peer_ip in peer_ips:
            try:
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_socket.connect((peer_ip, self.port))
                self.peers.append(client_socket)
                print(f"Connected to peer {peer_ip}")
                threading.Thread(target=self.listen_to_peer, args=(client_socket,)).start()
            except Exception as e:
                print(f"Could not connect to {peer_ip}: {e}")

    def listen_to_peer(self, conn):
        """Listen for incoming messages from a connected peer."""
        while self.running:
            try:
                message = conn.recv(1024).decode()
                if message:
                    print(f"Received: {message}")
                else:
                    break
            except ConnectionResetError:
                print("Peer disconnected")
                break
        conn.close()

    def broadcast_message(self, message):
        """Send a message to all connected peers."""
        for peer in self.peers:
            try:
                peer.send(message.encode())
            except Exception as e:
                print(f"Error sending to a peer: {e}")

    def stop(self):
        """Stop the server and clean up resources."""
        self.running = False
        if self.server:
            self.server.close()
        for peer in self.peers:
            peer.close()
        print("Node stopped.")



if __name__ == "__main__":

    robot = RandomRobotMove()
    tempSensor = TemperatureSensor()

    # Example usage
    host = "0.0.0.0"  # Use "0.0.0.0" to allow connections from any IP
    port = 12345
    node = PeerToPeerNode(host, port)

    # Start the server thread
    server_thread = threading.Thread(target=node.start_server, daemon=True)
    server_thread.start()

    # Connect to peers (add the IP addresses of other peers)
    # Get peer IPs to connect to
    peer_ips = input("Enter peer IPs (comma-separated, leave blank if none): ").strip().split(',')
    peer_ips = [ip.strip() for ip in peer_ips if ip.strip()]  # Clean the list
    node.connect_to_peers(peer_ips)

    # Queue to communicate temperature data between threads
    temp_queue = queue.Queue()

    def rotate_robot_thread():
        # print("Rotating robot in a separate thread...")
        robot.rotate_right()
    #
    # def forward_robot_thread():
    #     # print("Rotating robot in a separate thread...")
    #     robot.move_forward()
    #
    # def back_robot_thread():
    #     # print("Rotating robot in a separate thread...")
    #     robot.move_back()
    def random_robot_move():
        robot.random_move()

    def get_temp_thread():
        # print("Getting temperature in a separate thread...")
        temp_c_t = tempSensor.get_temp_c()
        temp_queue.put(temp_c_t)

    try:
        print("Type your message below or wait for incoming commands... (Type 'exit' to exit)")
        while True:
            # Non-blocking input using select
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)  # Timeout of 0.1 seconds

            # Threadsafe printing on main, Check for temperature updates from the queue
            while not temp_queue.empty():
                temp_c = temp_queue.get()
                print(f"Temperature (C): {temp_c}")
            if ready:
                message = sys.stdin.readline().strip()
                if message.lower() == "exit":
                    break
                if message.lower() == "random":
                    threading.Thread(target=random_robot_move, daemon=True).start()

                # if message.lower() == "rotate":
                #     # print("Rotating..")
                #     threading.Thread(target=rotate_robot_thread, daemon=True).start()
                # if message.lower() == "forward":
                #     # print("Rotating..")
                #     threading.Thread(target=forward_robot_thread, daemon=True).start()
                # if message.lower() == "back":
                #     # print("Rotating..")
                #     threading.Thread(target=back_robot_thread, daemon=True).start()
                if message.lower() == "get_temp":
                    threading.Thread(target=get_temp_thread, daemon=True).start()

                #TODO: Commenting temporarly as no brodcast, testing
                node.broadcast_message(message)

            # TODO: Commenting temporarily as no peers, testing
            # Check for messages in the queue
            if not node.message_queue.empty():
                message_code = node.message_queue.get()
                if message_code == 1:  # Rotate command received
                    print(f"message {message_code} received.")
    finally:
        node.stop()
