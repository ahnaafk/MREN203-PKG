import serial
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
PORT = '/dev/ttyACM0'
BAUD = 9600
class ArduinoInterface(Node):

    def __init__(self):
        super().__init__('arduino_interface')


        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        self.ser.reset_input_buffer()
        self.subscription = self.create_subscription(
            String,
            'motor_input',
            callback=self.serialWrite,
            qos_profile=10)
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.serialRead)
        self.i = 0

    #serialRead takes in odemetry data and formats it accordingly for the odemetry topic. 
    def serialRead(self):
        line = self.ser.readline().decode('utf-8').rstrip()
        
        #msg.data = line
        #TODO: parse the serial to check if it is odemetry data or not. if it is, publish it.
        #TODO: format the odemetry data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        

    #serialWrite will take in a string message and write it to the serial port.
    def serialWrite(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.ser.write(f'{msg.data}\n'.encode())
        self.get_logger().info('So I wrote: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    
    arduino_interface = ArduinoInterface()
    rclpy.spin(arduino_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arduino_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
