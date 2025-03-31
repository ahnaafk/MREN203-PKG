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
        
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        timer_period = 0.2  # seconds
        #self.timer = self.create_timer(timer_period, self.serialRead)
        self.i = 0
    def serialRead(self):
        """Reads odometry data from Arduino and publishes to cmd_vel"""
        msg = TwistStamped()
        
        try:
            line = self.ser.readline().decode('utf-8').strip()  # Read a full line
            
            if not line:  # Ignore empty reads
                return

            json_read = json.loads(line)  # Convert to dictionary
            
            if json_read.get('type') == 0.0:  # Check if valid odometry data
                msg.twist.linear.x = json_read.get("trans_v", 0.0)
                msg.twist.angular.z = json_read.get("angular_v", 0.0)
                self.publisher_.publish(msg)

            self.get_logger().info(f'Publishing: {json_read}')
            self.get_logger().info(f'Publishing: {msg}')
            self.get_logger().info(f'Publishing: {line}')
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid JSON received: {line}')

    def serialWrite(self, msg):
        """Takes in a Float32MultiArray and sends JSON to the Arduino"""
        data = list(msg.data)
        json_write = json.dumps({
            'type': round(data[0], 2) if len(data) > 0 else 0.0,
            'trans_v': round(data[1], 2) if len(data) > 1 else 1.0,
            'angular_v': round(data[2], 2) if len(data) > 2 else 2.0
        })

        self.ser.reset_output_buffer()  # Clear unsent data before writing
        self.ser.write((json_write + '\n').encode())  # Send JSON with newline
        
        self.get_logger().info(f'I heard: {json_write}')
        self.serialRead()
    #serialRead takes in odemetry data and formats it accordingly for the odemetry topic. 
"""    def serialRead(self):
        msg = TwistStamped()
        self.ser.reset_input_buffer()
        line = self.ser.readline().decode('utf-8').strip()
         json_read = json.loads(line) 
        if (json_read["type"] == 1):        
            msg.twist.linear.x = json_read["trans_v"]
            msg.twist.angular.z = json_read["angular_v"]
            self.publisher_.publish(msg)
        
        self.get_logger().info('Publishing: "%s"' % line)
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
       self.ser.reset_output_buffer()
       self.ser.write((json_write + '\n').encode())
       self.get_logger().info('I heard: "%s"' % json_write)
       #self.ser.write(f'{msg.data}\n'.encode())
       #self.get_logger().info('So I wrote: "%s"' % msg.data)
   """     

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
