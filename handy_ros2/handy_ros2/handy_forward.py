import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import Vector3, TransformStamped
from handy_ros2.utils import quaternion_from_euler

def calculateRotation(vector0: np.array, vector1: np.array):
    n = np.cross(vector0, vector1)
    if np.allclose(n, 0):
        return np.identity(3)

    u = n / np.linalg.norm(n)
    v = np.array([1.0, 0.0, 0.0])
    
    if np.allclose(u, v):
        return np.identity(3)
    elif np.allclose(u, -v):
        return np.array([
            [1, 0,  0],
            [0, -1, 0],
            [0, 0, -1]
        ])
    
    k = np.cross(u, v)
    c = np.dot(u, v)
    s = np.linalg.norm(k)

    kx, ky, kz = k
    k_skew = np.array([
        [0, -kz, ky],
        [kz, 0, -kx],
        [-ky, kx, 0]
    ])

    k_skew_squared = k_skew @ k_skew
    r = np.identity(3) + k_skew + k_skew_squared * ((1 - c) / (s ** 2))
    phi = np.deg2rad(5)
    A = np.array([
        [1, 0,  0],
        [0, np.cos(phi),  -np.sin(phi)],
        [0, np.sin(phi),  np.cos(phi)]
    ])
    r = A@r
    return r




def vec3ToNp(vector:Vector3)->np.array:
    x = vector.x
    y = vector.y
    z = vector.z
    return np.array([x,y,z])


class FrameListener(Node):
    def __init__(self):
        super().__init__('handy_forward')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_publisher = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.get_transform)

    def publish_transform(self, vector_i:np.array, vector_t:np.array):
        r = calculateRotation(vector_i,vector_t)
        vector_i = np.matmul(r,np.transpose(vector_i))
        vector_t = np.matmul(r,np.transpose(vector_t))


        q = quaternion_from_euler(0,0,0)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'
        t.child_frame_id = 'virtual_base'
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_publisher.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'virtual_base'
        t.child_frame_id = 'virtual_index'
        t.transform.translation.x = vector_i[0]
        t.transform.translation.y = vector_i[1]
        t.transform.translation.z = -vector_i[2]
        self.tf_publisher.sendTransform(t)
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'virtual_base'
        t.child_frame_id = 'virtual_thumb'
        t.transform.translation.x = vector_t[0]
        t.transform.translation.y = vector_t[1]
        t.transform.translation.z = -vector_t[2]
        self.tf_publisher.sendTransform(t)

    def get_transform(self):
        try:
            base_to_index_tip = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'index_tip',  # source_frame
                rclpy.time.Time()) 
            
            base_to_index_base = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'encoder_link_assembly',  # source_frame
                rclpy.time.Time())
            
            base_to_thumb = self.tf_buffer.lookup_transform(
                'base',
                'thumb_tip',
                rclpy.time.Time()
            )

            base_to_thumb_base = self.tf_buffer.lookup_transform(
                'base',
                'part_2',
                rclpy.time.Time()
            )


            
            o_to_f = vec3ToNp(base_to_index_tip.transform.translation)
            o_to_bf = vec3ToNp(base_to_index_base.transform.translation)
            b_to_f = o_to_f-o_to_bf
            o_to_t = vec3ToNp(base_to_thumb.transform.translation)
            bf_to_bt = -o_to_bf + o_to_t
            self.publish_transform(b_to_f,bf_to_bt)
            
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main():
    rclpy.init()
    node = FrameListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
