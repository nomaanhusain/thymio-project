import asyncio

PORT = 12345  # Port for communication
peers = []  # List of connected peer addresses


async def handle_connection(reader, writer):
    """Handles incoming connections and messages."""
    addr = writer.get_extra_info('peername')
    print(f"Connected by {addr}")
    peers.append((reader, writer))  # Keep track of peers

    try:
        while True:
            data = await reader.read(1024)
            if not data:
                print(f"Connection closed by {addr}")
                break

            message = data.decode()
            print(f"Received from {addr}: {message}")

            # Broadcast message to all peers except the sender
            for r, w in peers:
                if w != writer:
                    w.write(f"From {addr}: {message}".encode())
                    await w.drain()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        writer.close()
        await writer.wait_closed()
        peers.remove((reader, writer))


async def start_server():
    """Starts the server to listen for incoming connections."""
    server = await asyncio.start_server(handle_connection, '0.0.0.0', PORT)
    addr = server.sockets[0].getsockname()
    print(f"Server started on {addr}")

    async with server:
        await server.serve_forever()


async def connect_to_peers(peer_ips):
    """Connects to specified peer servers."""
    for peer_ip in peer_ips:
        try:
            reader, writer = await asyncio.open_connection(peer_ip, PORT)
            print(f"Connected to peer at {peer_ip}")
            peers.append((reader, writer))  # Keep track of peers

            # Start a task to receive messages from this peer
            asyncio.create_task(receive_from_peer(reader))
        except Exception as e:
            print(f"Failed to connect to {peer_ip}: {e}")


async def receive_from_peer(reader):
    """Handles receiving messages from a peer."""
    try:
        while True:
            data = await reader.read(1024)
            if not data:
                print("Peer connection closed.")
                break
            print(f"Received: {data.decode()}")
    except Exception as e:
        print(f"Error while receiving: {e}")


async def send_messages():
    """Allows the user to send messages to all peers."""
    while True:
        message = input("Enter message to send: ")
        for _, writer in peers:
            writer.write(message.encode())
            await writer.drain()


async def main():
    # Get peer IPs to connect to
    peer_ips = input("Enter peer IPs (comma-separated, leave blank if none): ").strip().split(',')
    peer_ips = [ip.strip() for ip in peer_ips if ip.strip()]  # Clean the list

    # Start server and connect to peers simultaneously
    server_task = asyncio.create_task(start_server())
    connect_task = asyncio.create_task(connect_to_peers(peer_ips))

    # Start message sending
    send_task = asyncio.create_task(send_messages())

    # Wait for all tasks
    await asyncio.gather(server_task, connect_task, send_task)


if __name__ == "__main__":
    asyncio.run(main())
