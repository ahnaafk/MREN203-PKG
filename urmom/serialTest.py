import serial
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import json

PORT = '/dev/ttyACM0'
BAUD = 115200
class ArduinoInterface(Node):

    def __init__(self):
        super().__init__('arduino_interface')


        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        self.ser.reset_input_buffer()
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_input',
            callback=self.serialWrite,
            qos_profile=10)
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(String, 'odom', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.serialRead)
        self.i = 0

    #serialRead takes in odemetry data and formats it accordingly for the odemetry topic. 
    def serialRead(self):
        msg = String()
        #line = self.ser.readline().decode('utf-8').rstrip()
        #test = json.loads(line) 
        #msg.data = line
        #TODO: parse the serial to check if it is odemetry data or not. if it is, publish it.
        #TODO: format the odemetry data
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        

    #serialWrite will take in a string message and write it to the serial port.
    def serialWrite(self, msg):
       data = list(msg.data)
       data[0] = round(data[0], 2)
       data[1] = round(data[1], 2)
       data[2] = round(data[2], 2)
       json_write = json.dumps({
           'type':data[0] if len(data) > 0 else 0.0,
           'trans_v':data[1] if len(data) > 1 else 1.0,
           'angular_v':data[2] if len(data) > 2 else 2.0
       })
       self.ser.write((json_write + '|').encode())
       self.get_logger().info('I heard: "%s"' % json_write)
       #self.ser.write(f'{msg.data}\n'.encode())
       #self.get_logger().info('So I wrote: "%s"' % msg.data)
        

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
