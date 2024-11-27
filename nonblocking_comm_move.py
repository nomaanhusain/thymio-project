import socket
import sys
import threading
import time
from thymiodirect import Connection
from thymiodirect import Thymio
import queue
import select


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


robot = None


def establish_robot_connection():
    global robot
    port = Connection.serial_default_port()
    th = Thymio(serial_port=port,
                on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))
    # Connect to Robot
    th.connect()
    robot = th[th.first_node()]

    # Delay to allow robot initialization of all variables
    time.sleep(1)
    # b) print all variables
    print(th.variables(th.first_node()))
    print("Robot connected")


def rotate_robot():
    global robot
    if robot is not None:
        counter = 10000
        print("Rotate 180")
        while counter > 0:
            if counter % 1000 == 0:
                print(f"Rotation={counter}")
            robot['motor.left.target'] = 200
            robot['motor.right.target'] = -200
            counter -= 1
        else:
            print("robot stop")
            robot['motor.left.target'] = 0
            robot['motor.right.target'] = 0


if __name__ == "__main__":

    establish_robot_connection()

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

    try:
        print("Type your message below or wait for incoming commands...")
        while True:
            # Non-blocking input using select
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)  # Timeout of 0.1 seconds
            if ready:
                message = sys.stdin.readline().strip()
                if message.lower() == "exit":
                    break
                node.broadcast_message(message)

            # Check for messages in the queue
            if not node.message_queue.empty():
                message_code = node.message_queue.get()
                if message_code == 1:  # Rotate command received
                    print("Rotate command received in main thread. Calling rotate function.")
                    rotate_robot()
    finally:
        node.stop()

    # try:
    #     while True:
    #         if not node.message_queue.empty():
    #             message_code = node.message_queue.get()
    #             if message_code == 1:
    #                 print("Rotate robot message received")
    #                 rotate_robot()
    #
    #         # Input messages to broadcast
    #         message_input = input("Enter message to send (type 'exit' to quit): ")
    #         if message_input.lower() == "exit":
    #             break
    #         node.broadcast_message(message_input)
    # finally:
    #     node.stop()
