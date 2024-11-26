import socket
import threading
import time
from thymiodirect import Connection
from thymiodirect import Thymio


class PeerToPeerNode:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.peers = []  # List of peer IPs
        self.server = None
        self.running = True

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
                        self.rotate_robot()
                else:
                    break
            except ConnectionResetError:
                print(f"Peer {addr} disconnected")
                break
        conn.close()

    def rotate_robot(self):
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
        counter = 10
        while counter > 0:
            robot['motor.left.target'] = 100
            robot['motor.right.target'] = -100
            counter -= 1
        else:
            robot['motor.left.target'] = 0
            robot['motor.right.target'] = 0


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
        while True:
            # Input messages to broadcast
            message_input = input("Enter message to send (type 'exit' to quit): ")
            if message_input.lower() == "exit":
                break
            node.broadcast_message(message_input)
    finally:
        node.stop()
