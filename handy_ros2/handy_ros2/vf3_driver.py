import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import Vector3, PoseStamped, Point
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

PINKY_START = -1.08
PINKY_END = -1.90
MIDDLE = (PINKY_START+PINKY_END)/2
ADJUST_RANGE = 45
ADJUST_RATIO = ADJUST_RANGE/(PINKY_START-PINKY_END)

def vec3ToNp(vector:Vector3)->np.array:
    x = vector.x
    y = vector.y
    z = vector.z
    return np.array([x,y,z])


class vf3Driver(Node):
    def __init__(self):
        super().__init__('handy_forward')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.drive_angles)
        self.publisher_ = self.create_publisher(PoseStamped,"/vf3_manipulator/setPose",1)
        self.pinky_adjust = 0


    def drive_angles(self):#
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'thumb'
        coords = self.get_coords_thumb()
        if coords:
            msg.pose.position = coords
            self.publisher_.publish(msg)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'index'
        coords = self.get_coords_index()
        if coords:
            msg.pose.position = coords
            self.publisher_.publish(msg)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'pinky'
        coords = self.get_coords_pinky()
        if coords:
            msg.pose.position = coords
            self.publisher_.publish(msg)
        
        
    def get_coords_index(self):
        msg = Point()
        try:
            base_to_index = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'index_tip',  # source_frame
                rclpy.time.Time())

            base_to_index_base = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'encoder_link_assembly__configuration_default',  # source_frame
                rclpy.time.Time()) 

            b_to_i = vec3ToNp(base_to_index.transform.translation)
            b_to_ib = vec3ToNp(base_to_index_base.transform.translation)
            ib_to_i = -b_to_i+b_to_ib
            ib_to_i = ib_to_i*1000 #meteres to millimeters

            ib_to_i[2]=ib_to_i[2]*0.6# y scaling
            ib_to_i[1]=(-ib_to_i[1])*0.8 # x translation#
            msg.x = ib_to_i[2]
            msg.y = ib_to_i[1]
            return msg
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')
    
    def get_coords_thumb(self):
        msg = Point()
        try:
            base_to_index = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'thumb_tip',  # source_frame
                rclpy.time.Time())

            base_to_index_base = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'encoder_link_assembly__configuration_default_2',  # source_frame
                rclpy.time.Time()) 

            b_to_i = vec3ToNp(base_to_index.transform.translation)
            b_to_ib = vec3ToNp(base_to_index_base.transform.translation)
            ib_to_i = -b_to_i+b_to_ib
            ib_to_i = ib_to_i*1000 #meteres to millimeters

            ib_to_i[2]=ib_to_i[2]*0.6# y scaling
            ib_to_i[1]=(-ib_to_i[1])*0.8 # x translation#
            msg.x = ib_to_i[2]
            msg.y = ib_to_i[1]
            return msg
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

    def get_coords_pinky(self):
        msg = Point()
        try:
            base_to_index = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'pinky_tip',  # source_frame
                rclpy.time.Time())

            base_to_index_base = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'encoder_link_assembly__configuration_default_3',  # source_frame
                rclpy.time.Time()) 

            b_to_i = vec3ToNp(base_to_index.transform.translation)
            b_to_ib = vec3ToNp(base_to_index_base.transform.translation)
            ib_to_i = -b_to_i+b_to_ib
            ib_to_i = ib_to_i*1000 #meteres to millimeters

            ib_to_i[2]=ib_to_i[2]*0.6# y scaling
            ib_to_i[1]=(-ib_to_i[1])*0.8 # x translation#
            msg.x = ib_to_i[2]
            msg.y = ib_to_i[1]
            return msg
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main():
    rclpy.init()
    node = vf3Driver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
