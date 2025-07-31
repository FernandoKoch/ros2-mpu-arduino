import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np

class ImuPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('imu_pose_node')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)
        self.publisher_ = self.create_publisher(PoseStamped, 'pose_estimate', 10)
        self.prev_time = None
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # Quaternion x, y, z, w

    def imu_callback(self, msg):
        # Timestamp
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = t
            return
        dt = t - self.prev_time
        self.prev_time = t

        # Integrate acceleration to get velocity and position
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.velocity += acc * dt
        self.position += self.velocity * dt

        # Use IMU orientation if available (otherwise keep previous)
        if msg.orientation.w != 0.0:
            self.orientation = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        pose_msg.pose.orientation.x = self.orientation[0]
        pose_msg.pose.orientation.y = self.orientation[1]
        pose_msg.pose.orientation.z = self.orientation[2]
        pose_msg.pose.orientation.w = self.orientation[3]

        self.publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPoseEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()