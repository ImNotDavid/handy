import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class FrameListener(Node):
    def __init__(self):
        super().__init__('handy_forward')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.get_transform)

    def get_transform(self):
        try:
            index_tip = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'index_tip',  # source_frame
                rclpy.time.Time()) 
            
            thumb_tip = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'thumb_tip',  # source_frame
                rclpy.time.Time())
            
            index_base = self.tf_buffer.lookup_transform(
                'base',  # target_frame
                'encoder_link_assembly',  # source_frame
                rclpy.time.Time())
            print(index_tip.transform.translation ,thumb_tip.transform.translation ,index_base.transform.translation )
            
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main():
    rclpy.init()
    node = FrameListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
