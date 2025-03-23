import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
    
        self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
        self.subscription = self.create_subscription(
            String,
            'topic',
            callback=self.listener_callback,
            qos_profile=10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.5
        #self.timer = self.createtimer(timer_period, self.listener_callback)
 
    #this will be what writes to serial. 
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.ser.write(f'{msg.data}\n'.encode())
        #line = self.ser.readline().decode('utf-8').rstrip()
        self.get_logger().info('So I wrote: "%s"' % msg.data)


def main(args=None):
    ##startup cod
    
    ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
    ser.reset_input_buffer()
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
