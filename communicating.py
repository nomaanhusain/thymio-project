import asyncio

PORT = 12345  # Port for communication


async def handle_connection(reader, writer):
    """Handles incoming connections and messages."""
    addr = writer.get_extra_info('peername')
    print(f"Connected by {addr}")

    try:
        while True:
            # Read messages
            data = await reader.read(1024)
            if not data:
                print(f"Connection closed by {addr}")
                break

            message = data.decode()
            print(f"Received from {addr}: {message}")

            # Optional: Echo the message back
            response = f"Echo: {message}"
            writer.write(response.encode())
            await writer.drain()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        writer.close()
        await writer.wait_closed()


async def start_server():
    """Starts the server to listen for incoming connections."""
    server = await asyncio.start_server(handle_connection, '0.0.0.0', PORT)
    addr = server.sockets[0].getsockname()
    print(f"Server started on {addr}")

    async with server:
        await server.serve_forever()


async def start_client(remote_ip):
    """Connects to a remote server and allows sending messages."""
    reader, writer = await asyncio.open_connection(remote_ip, PORT)
    print(f"Connected to server at {remote_ip}")

    try:
        while True:
            # Send messages
            message = input("Enter message: ")
            writer.write(message.encode())
            await writer.drain()

            # Read server response
            response = await reader.read(1024)
            print(f"Server response: {response.decode()}")
    except KeyboardInterrupt:
        print("Closing connection.")
    finally:
        writer.close()
        await writer.wait_closed()


async def peer_node():
    """Run both server and client concurrently."""
    server_task = asyncio.create_task(start_server())

    # Connect to existing peers (optional)
    connect_to_peers = input("Do you want to connect to peers? (yes/no): ").strip().lower()
    if connect_to_peers == 'yes':
        remote_ip = input("Enter peer IP: ").strip()
        client_task = asyncio.create_task(start_client(remote_ip))
        await asyncio.gather(server_task, client_task)
    else:
        await server_task


if __name__ == "__main__":
    asyncio.run(peer_node())
