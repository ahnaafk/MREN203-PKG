import keyboard
import time
import json
import pathlib

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String

class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'motor_input', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        v_d = 0.0
        w_d = 0.0
        if keyboard.is_pressed('w'):
            v_d = 1.0
        if keyboard.is_pressed('a'):
            w_d = 1.0
        if keyboard.is_pressed('s'):
            v_d = -1.0
        if keyboard.is_pressed('d'):
            w_d = -1.0
        msg.data = [0.0, v_d , w_d]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)




# def checkKeyPress():
#     if keyboard.is_pressed('w'):
#         print('w')
#     if keyboard.is_pressed('a'):
#         print('a')
#     if keyboard.is_pressed('s'):
#         print('s')
#     if keyboard.is_pressed('d'):
#         print('d')

""""
1. Create a publisher that publishes the keyboard input
  a. interface with Float32MultiArray
  b. Publish the data to the topic
"""

# def serialWrite(self, msg):
#         """Takes in a Float32MultiArray and sends JSON to the Arduino"""
#         data = list(msg.data)
#         json_write = json.dumps({
#             'type': round(data[0], 2) if len(data) > 0 else 0.0,
#             'trans_v': round(data[1], 2) if len(data) > 1 else 1.0,
#             'angular_v': round(data[2], 2) if len(data) > 2 else 2.0
#         })

#         self.ser.reset_output_buffer()  # Clear unsent data before writing
#         self.ser.write((json_write + '\n').encode())  # Send JSON with newline
        
#         self.get_logger().info(f'I heard: {json_write}')
#         self.serialRead()

def main(args=None):
    rclpy.init(args=args)
    #message = String()
    #message.data = "ur mom" 
    keyboard_publisher = KeyboardPublisher()
    rclpy.spin(keyboard_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
