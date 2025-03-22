import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        ser.reset_input_buffer()
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback(ser=ser),
            10)
        self.subscription  # prevent unused variable warning
    
    #this will be what writes to serial. 
    def listener_callback(self, msg, ser):
        self.get_logger().info('I heard: "%s"' % msg.data)
        ser.write(f'{msg.data}\n'.encode())
        line = ser.readline().decode('utf-8').rstrip()
        self.get_logger().info('So I wrote: "%s"' % line)


def main(args=None):
    ##startup code
    rclpy.init(args=args)
    
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()