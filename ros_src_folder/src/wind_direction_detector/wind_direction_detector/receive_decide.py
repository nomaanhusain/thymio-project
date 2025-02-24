import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReceiveDecide(Node):
    def __init__(self):
        super().__init__('receive_decide')
        self.subscription = self.create_subscription(
            String,
            'wind_direction',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        id = msg.data.split(':')[0]
        angle = msg.data.split(':')[1]

        print(f'Received from {id}, angle {angle}')

def main(args=None):
    rclpy.init(args=args)

    receive_decide = ReceiveDecide()

    try:
        rclpy.spin(receive_decide)
    except KeyboardInterrupt:
        receive_decide.get_logger().info("Shutting down node...")
    finally:
        receive_decide.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
