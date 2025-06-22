import rclpy
from dpali_msgs.msg import DPaliCoordPair
import numpy as np
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import Vector3, Pose2D
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import joblib
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import tensorflow as tf
from tensorflow.keras import layers, models
import pandas as pd


path = f"software/learning/models/"


def vec3ToNp(vector: Vector3) -> np.array:
    x = vector.x
    y = vector.y
    z = vector.z
    return np.array([x, y, z])


class DPaliML(Node):
    def __init__(self):
        super().__init__("dpali_ml")
        msg = DPaliCoordPair()
        msg.left.x = float(-23)
        msg.left.y = float(73)
        msg.right.x = float(25)
        msg.right.y = float(67)
        self.current_coords = msg
        self.model = joblib.load(path+'orientation_model.pkl')
        self.scalar_X = joblib.load(path+'scaler_X.pkl')
        self.scalar_y = joblib.load(path+'scaler_y.pkl')

        self.target_pose = Pose2D()
        self.block_pose = Pose2D()
        self.block_subscriber = self.create_subscription(
            Pose2D, "/learning/block_pose", self.get_block, 1
        )
        self.target_subscriber = self.create_subscription(
            Pose2D, "/learning/target_pose", self.get_target, 1
        )
        self.timer = self.create_timer(0.1, self.drive_angles)
        self.publisher_ = self.create_publisher(DPaliCoordPair, "/dpali/set_coords", 1)

    def drive_angles(self):  #

        dlx, dly, drx, dry = self.predict_d_pali(self.block_pose.x,self.block_pose.y,self.block_pose.theta,self.target_pose.theta)

        self.current_coords.left.x +=  np.clip(float(dlx),-1,1)
        self.current_coords.left.y += np.clip(float(dly),-1,1)
        self.current_coords.right.x += np.clip(float(drx),-1,1)
        self.current_coords.right.y += np.clip(float(dry),-1,1)
                

        print(f'Target_orientation: {self.target_pose.theta}, Current Orientation: {self.block_pose.theta}')
        self.publisher_.publish(self.current_coords)

    def get_target(self, msg: Pose2D):
        self.target_pose = msg

    def get_block(self, msg: Pose2D):
        self.block_pose = msg

    def predict_d_pali(self, block_x, block_y, block_orientation, target_orientation):
        orientation_delta = target_orientation-block_orientation
        msg = self.current_coords
        X = pd.DataFrame([[msg.left.x,msg.left.y,msg.right.x,msg.right.y,block_x, block_y, orientation_delta]],
        columns=["Left_x_setpoint", "Left_y_setpoint", "Right_x_setpoint", "Right_y_setpoint",'Block_x', 'Block_y', 'Orientation_Difference'])
        X_scaled = self.scalar_X.transform(X)
        pred_scaled = self.model.predict(X_scaled)
        return self.scalar_y.inverse_transform(pred_scaled)[0]

def main():
    rclpy.init()
    node = DPaliML()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
