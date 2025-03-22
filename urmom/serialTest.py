import Serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
                            self.publisher_ = self.create_publisher(String, 'topic', 10)
                                    timer_period = 0.5  # seconds
                                            self.timer = self.create_timer(timer_period, self.timer_callback)
                                                    self.i = 0

                                                        def timer_callback(self):
                                                            msg = String()
                                                                            msg.data = 'Hello World: %d' % self.i
                                                                                    self.publisher_.publish(msg)
                                                                                            self.get_logger().info('Publishing: "%s"' % msg.data)
                                                                                                    self.i += 1

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            ser.reset_input_buffer()

                while True:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8').rstrip()
                                                print(line)
