import rclpy
from dpali_msgs.msg import DPaliCoordPair
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose2D
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import csv
import time
import os

TRIAL_TYPE = "orientation_learning"
path = f"software/output/{TRIAL_TYPE}"


def vec3ToNp(vector: Vector3) -> np.array:
    x = vector.x
    y = vector.y
    z = vector.z
    return np.array([x, y, z])


def get_file_no(path):
    trial_no = 0
    while os.path.exists(f"{path}/data_{trial_no}.csv"):
        trial_no = trial_no + 1
    return trial_no


class LearningLogger(Node):
    def __init__(self):
        super().__init__("learning_logger")
        self.setpoints = DPaliCoordPair()
        self.target_pose = Pose2D()
        self.block_pose = Pose2D()
        self.setpoint_subscriber = self.create_subscription(
            DPaliCoordPair, "/dpali/set_coords", self.get_setpoints, 1
        )
        self.block_subscriber = self.create_subscription(
            Pose2D, "/learning/block_pose", self.get_block, 1
        )
        self.target_subscriber = self.create_subscription(
            Pose2D, "/learning/target_pose", self.get_target, 1
        )
        self.timer = self.create_timer(0.01, self.write_log)
        trial_no = get_file_no(path)
        csvfile = open(f"{path}/data_{trial_no}.csv", "w", newline="")
        self.spamwriter = csv.writer(
            csvfile, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL
        )
        self.spamwriter.writerow(
            [
                "Timestamp",
                "Block_x",
                "Block_y",
                "Block_orientation",
                "Target_x",
                "Target_y",
                "Target_orientation",
                "Left_x_setpoint",
                "Left_y_setpoint",
                "Right_x_setpoint",
                "Right_y_setpoint",
            ]
        )

    def write_log(self):  #
        try:
            self.spamwriter.writerow(
                [
                    round(time.time() * 1000),
                    self.block_pose.x,
                    self.block_pose.y,
                    self.block_pose.theta,
                    self.target_pose.x,
                    self.target_pose.y,
                    self.target_pose.theta,
                    self.setpoints.left.x,
                    self.setpoints.left.y,
                    self.setpoints.right.x,
                    self.setpoints.right.y,
                ]
            )
        except Exception as e:
            self.get_logger().warn(f'Could not log: {e}')

    def get_target(self, msg: Pose2D):
        self.target_pose = msg

    def get_block(self, msg: Pose2D):
        self.block_pose = msg

    def get_setpoints(self, msg: DPaliCoordPair):
        self.setpoints = msg


def main():
    rclpy.init()
    node = LearningLogger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
