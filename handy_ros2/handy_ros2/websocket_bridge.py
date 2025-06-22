import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets
import numpy as np
import json
from handy_ros2.utils import quaternion_from_euler
from geometry_msgs.msg import  TransformStamped, Pose2D
import threading
from tf2_ros import TransformBroadcaster


WS_SERVER_URI = "ws://0.0.0.0:8765"  

class WebSocketROS2Bridge(Node):
    def __init__(self):
        super().__init__('websocket_ros2_bridge')
        self.tf_publisher = TransformBroadcaster(self)
        self.target_publisher = self.create_publisher(Pose2D,'/learning/target_pose',1)
        self.block_publisher = self.create_publisher(Pose2D,'/learning/block_pose',1)
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        asyncio.run_coroutine_threadsafe(self.websocket_listener(), self.loop)

    async def websocket_listener(self):
        try:
            async with websockets.connect(WS_SERVER_URI) as websocket:
                self.get_logger().info(f"Connected to WebSocket server at {WS_SERVER_URI}")
                while True:
                    message = await websocket.recv()
                    msg = String()
                    msg.data = message
                    data = json.loads(message)
                    object_angle = data['block_orientation']
                    target_angle = data['target_orientation']
                    q = quaternion_from_euler(np.deg2rad(object_angle),0,0)
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'virtual_base'
                    t.child_frame_id = 'virtual_object'
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = data['block_y']/1000
                    t.transform.translation.z = -data['block_x']/1000
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
                    self.tf_publisher.sendTransform(t)
                    
                    msg = Pose2D()
                    msg.x = data['block_x']
                    msg.y = data['block_y']
                    msg.theta = data['block_orientation']
                    self.block_publisher.publish(msg)

                    msg = Pose2D()
                    msg.x = data['target_x']
                    msg.y = data['target_y']
                    msg.theta = data['target_orientation']
                    self.target_publisher.publish(msg)



                    
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketROS2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()