# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import numpy as np
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from handy_ros2.utils import quaternion_from_euler
import math
import serial
import time
import struct

# Replace with your ESP32's serial port (e.g., 'COM3' for Windows, '/dev/ttyUSB0' for Linux/macOS)
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200  # Match the baud rate of your ESP32





class JointBridge(Node):

    def __init__(self):
        super().__init__('handy_bridge')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_publisher = self.create_publisher(TFMessage, 'tf', 10)
        timer_period = 0.11  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.5)  # Allow time for the connection to establish

    print("Reading data from ESP32...")

    def timer_callback(self):
        joint_angle, q = self.get_angles()
        if joint_angle:
            msg = JointState()
            msg.name = ['thumb_0','thumb_1','thumb_2', 'thumb_3', 'index_0', 'index_1','index_2']
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = [math.radians(float(i)) for i in joint_angle] 
            msg.velocity = []
            msg.effort=[]
            self.publisher_.publish(msg)

            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.child_frame_id = "base"
            msg._transform.rotation.x = q[0]  
            msg._transform.rotation.y = q[1]  
            msg._transform.rotation.z = q[2]  
            msg._transform.rotation.w = q[3]  
            outMsg = TFMessage()
            outMsg.transforms = [msg]
            self.tf_publisher.publish(outMsg)
            #self.get_logger().info('Publishing: "%s"' % msg.position)



    def get_angles(self):
        angles = None
        quaternion = None
        while self.ser.in_waiting >= 42:
            byte = self.ser.read(1)
            if byte == b'\xAA':
                data = self.ser.read(40)
                end = self.ser.read(1)
                if end == b'\x55':
                    try:
                        values = struct.unpack('10f', data)
                        angles = values[:7]
                        orientation = values[7:]
                        orientation = [math.radians(i) for i in orientation]
                        quaternion = quaternion_from_euler(orientation[1],-orientation[0],orientation[2])
                        angles = list(angles)
                        angles[0] = -angles[0]
                        angles[5] = -angles[5]
                        angles[2] = -angles[2]
                        angles[3] = -45.0
                    except struct.error:
                        pass
                break
        print(angles)
        self.ser.reset_input_buffer()
        return angles,quaternion

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = JointBridge()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
