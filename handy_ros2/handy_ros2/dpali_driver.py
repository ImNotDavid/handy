import rclpy
from dpali_msgs.msg import DPaliCoordPair
import numpy as np
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import Vector3, TransformStamped
from std_msgs.msg import Header


def vec3ToNp(vector:Vector3)->np.array:
    x = vector.x
    y = vector.y
    z = vector.z
    return np.array([x,y,z])


class DPaliDriver(Node):
    def __init__(self):
        super().__init__('handy_forward')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.drive_angles)
        self.publisher_ = self.create_publisher(DPaliCoordPair,"/dpali/set_coords",1)

    def drive_angles(self):#
        coords = self.get_coords_index()
        msg = DPaliCoordPair()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.right.x = coords[0]
        msg.right.y = coords[1]
        coords = self.get_coords_thumb()
        msg.left.x = coords[0]
        msg.left.y = coords[1]

        self.publisher_.publish(msg)

        
    def get_coords_index(self):
        try:
            base_to_index = self.tf_buffer.lookup_transform(
                'virtual_base',  # target_frame
                'virtual_index',  # source_frame
                rclpy.time.Time()) 
            b_to_i = vec3ToNp(base_to_index.transform.translation)
            b_to_i = b_to_i*1000 #meteres to millimeters

            b_to_i[2]=b_to_i[2]*0.6# y scaling
            b_to_i[1]=(-b_to_i[1])*0.6 # x translation#
            return (b_to_i[1],b_to_i[2])
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')
    
    def get_coords_thumb(self):
        try:
            base_to_index = self.tf_buffer.lookup_transform(
                'virtual_base',  # target_frame
                'virtual_thumb',  # source_frame
                rclpy.time.Time()) 
            b_to_i = vec3ToNp(base_to_index.transform.translation)
            b_to_i = b_to_i*1000 #meteres to millimeters

            b_to_i[2]=b_to_i[2]*0.6# y scaling
            b_to_i[1]=(-b_to_i[1])*0.6 # x translation#
            return (b_to_i[1],b_to_i[2])
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main():
    rclpy.init()
    node = DPaliDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
