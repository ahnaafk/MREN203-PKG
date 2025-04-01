import serial
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math
import json

PORT = '/dev/ttyACM1'
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

          # Odom Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
 #       self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Simulated robot state
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.w = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Timer to update odometry
        
    def publishOdom(self):
        #Compute robot velocity
        VERBOSE = False
        v = self.v
        omega = self.w
        
        

        """
        # Publish Odometry message
        """
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega
        
        if (VERBOSE == True): 
            self.get_logger().info(f'v: {v}')
            self.get_logger().info(f'omega: {omega}')
            self.get_logger().info(f'odometry: {odom_msg}')

        self.odom_pub.publish(odom_msg)


    def serialRead(self):
        """Reads odometry data from Arduino and calls publishOdom"""
        VERBOSE = False
        
        try:
            line_raw = self.ser.readline().decode('utf-8').strip()  # Read a full line
            line = line_raw[36:]
            
            if not line:  # Ignore empty reads
                return

            json_read = json.loads(line)  # Convert to dictionary
            
            if json_read.get('type') == 0.0:  # Check if valid odometry data
                self.v = json_read.get("trans_v", 0.0)
                self.w = json_read.get("angular_v", 0.0)

            if (VERBOSE == True): 
                self.get_logger().info(f'Publishing: {json_read}')
                self.get_logger().info(f'trans_vd: {self.v}')
                self.get_logger().info(f'angular_vd: {self.w}')
                self.get_logger().info(f'Serial RAW: {line}')
            
            #once it reads in the correct data, publish odom 
            self.publishOdom()

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
