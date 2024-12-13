import csv
import socket

ip_list = [item for row in csv.reader(open('csv_151.csv')) for item in row]

print(ip_list)

hostname = socket.gethostname()
ip_address = socket.gethostbyname(hostname)
print(ip_address)
last_add = ip_address.split('.')[-1]
print(last_add)


def get_physical_ip():
    try:
        # Connect to an external host to get the local network IP
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))  # Using Google's DNS server
            ip_address = s.getsockname()[0]
        return ip_address
    except Exception as e:
        return f"Error: {e}"


# Get and print the physical IP address
physical_ip = get_physical_ip()
print(f"Physical IP Address: {physical_ip}")

# Extract the last part of the IP address
last_part = physical_ip.split('.')[-1]
print(f"Last part of Physical IP Address: {last_part}")
