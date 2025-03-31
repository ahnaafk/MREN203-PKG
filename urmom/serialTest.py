import serial
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
#import tf_transformations
#import tf2_ros
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
        
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        timer_period = 0.2  # seconds
        #self.timer = self.create_timer(timer_period, self.serialRead)
        self.i = 0
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
        # self.create_timer(0.05, self.update_odometry)  # 20 Hz

    def serialRead(self):
        """Reads odometry data from Arduino and publishes to cmd_vel"""
        # msg = TwistStamped()
        
        try:
            line = self.ser.readline().decode('utf-8').strip()  # Read a full line
            line = line[36:]
            
            if not line:  # Ignore empty reads
                return

            json_read = json.loads(line)  # Convert to dictionary
            
            if json_read.get('type') == 0.0:  # Check if valid odometry data
                self.v = json_read.get("trans_v", 0.0)
                self.w = json_read.get("angular_v", 0.0)

            self.get_logger().info(f'Publishing: {json_read}')
            #self.get_logger().info(f'trans_vd: {self.v}')
            #self.get_logger().info(f'angular_vd: {self.w}')
            # self.get_logger().info(f'Publishing: {msg}')
            #self.get_logger().info(f'Serial RAW: {line}')
            
            #current_time = self.get_clock().now()
            #dt = (current_time - self.last_time).nanoseconds / 1e9
            #self.last_time = current_time

            # Compute robot velocity
            """v = self.v
            omega = self.w

            # Update robot position
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            self.theta += omega * dt
            """
            # Publish Odometry message
            """
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation.x = odom_quat[0]
            odom_msg.pose.pose.orientation.y = odom_quat[1]
            odom_msg.pose.pose.orientation.z = odom_quat[2]
            odom_msg.pose.pose.orientation.w = odom_quat[3]
            self.odom_pub.publish(odom_msg)

            # Publish TF transformation
            odom_tf = TransformStamped()
            odom_tf.header.stamp = current_time.to_msg()
            odom_tf.header.frame_id = "odom"
            odom_tf.child_frame_id = "base_link"
            odom_tf.transform.translation.x = self.x
            odom_tf.transform.translation.y = self.y
            odom_tf.transform.rotation.x = odom_quat[0]
            odom_tf.transform.rotation.y = odom_quat[1]
            odom_tf.transform.rotation.z = odom_quat[2]
            odom_tf.transform.rotation.w = odom_quat[3]"""
            #self.tf_broadcaster.sendTransform(odom_tf)
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
