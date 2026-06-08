#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.time import Time
from collections import deque
import numpy as np
import tf2_ros

class ErrorStateEKF(Node):
    def __init__(self):
        super().__init__('error_state_ekf')

        # EKF error state: [ex, ey, eyaw]
        self.state = np.zeros(3)
        self.P = np.eye(3) * 0.01  # initial error covariance
        self.Q = np.diag([0.001, 0.001, 0.001])  # process noise

        # Buffers for interpolation
        self.imu_buffer = deque(maxlen=50)
        self.lidar_buffer = deque(maxlen=20)

        # Last EKF update time
        self.t_last = None

        # Subscribers
        self.create_subscription(Odometry, 'ai_odom', self.imu_callback, 50)
        self.create_subscription(Odometry, 'lidar_odom', self.lidar_callback, 20)

        # Publisher
        self.pub = self.create_publisher(Odometry, 'odom_fused', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # EKF timer (20–50Hz)
        self.timer = self.create_timer(0.05, self.ekf_loop)

    # ---------------- Callbacks ----------------
    def imu_callback(self, msg):
        t = Time.from_msg(msg.header.stamp)
        self.imu_buffer.append((t, msg))

    def lidar_callback(self, msg):
        t = Time.from_msg(msg.header.stamp)
        self.lidar_buffer.append((t, msg))

    # ---------------- Helpers ----------------
    @staticmethod
    def angle_diff(a, b):
        diff = a - b
        while diff > np.pi:
            diff -= 2*np.pi
        while diff < -np.pi:
            diff += 2*np.pi
        return diff

    def interpolate_odom(self, msg1, t1, msg2, t2, t_now):
        dt_total = (t2 - t1).nanoseconds / 1e9
        dt_now = (t_now - t1).nanoseconds / 1e9
        ratio = dt_now / dt_total if dt_total != 0 else 0.0

        x = msg1.pose.pose.position.x + ratio * (msg2.pose.pose.position.x - msg1.pose.pose.position.x)
        y = msg1.pose.pose.position.y + ratio * (msg2.pose.pose.position.y - msg1.pose.pose.position.y)

        _, _, yaw1 = euler_from_quaternion([msg1.pose.pose.orientation.x,
                                            msg1.pose.pose.orientation.y,
                                            msg1.pose.pose.orientation.z,
                                            msg1.pose.pose.orientation.w])
        _, _, yaw2 = euler_from_quaternion([msg2.pose.pose.orientation.x,
                                            msg2.pose.pose.orientation.y,
                                            msg2.pose.pose.orientation.z,
                                            msg2.pose.pose.orientation.w])
        yaw = yaw1 + ratio * self.angle_diff(yaw2, yaw1)

        odom = Odometry()
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = msg1.pose.pose.position.z
        q = quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.header.frame_id = msg1.header.frame_id
        odom.child_frame_id = msg1.child_frame_id
        odom.header.stamp = msg1.header.stamp
        odom.pose.covariance = msg1.pose.covariance
        return odom

    def get_interpolated_msg(self, buffer, t_now):
        if len(buffer) < 2:
            return None
        for i in range(len(buffer)-1):
            t1, msg1 = buffer[i]
            t2, msg2 = buffer[i+1]
            if t1 <= t_now <= t2:
                return self.interpolate_odom(msg1, t1, msg2, t2, t_now)
        # fallback: last message
        return buffer[-1][1]

  
    # EKF2----------------------------------------------------------------------------------
    def ekf_loop(self):
        if len(self.imu_buffer) < 2 or len(self.lidar_buffer) < 1:
            return

        # Use latest IMU time as reference for prediction
        t_now = self.imu_buffer[-1][0]

        if self.t_last is None:
            self.t_last = t_now
            return

        dt = (t_now - self.t_last).nanoseconds / 1e9
        if dt <= 0:
            return

        # ---------------- Prediction ----------------
        imu_now = self.get_interpolated_msg(self.imu_buffer, t_now)
        imu_last = self.get_interpolated_msg(self.imu_buffer, self.t_last)
        if imu_now is None or imu_last is None:
            return

        # <<< added: extract velocity covariance from IMU msg (if Odometry type)
        var_vx_imu, var_vy_imu = None, None
        try:
            cov_twist = np.array(imu_now.twist.covariance).reshape(6,6)
            var_vx_imu = cov_twist[0,0]
            var_vy_imu = cov_twist[1,1]
        except Exception:
            # fallback conservative defaults if not available
            var_vx_imu, var_vy_imu = 0.1, 0.1

        # <<< added: set process noise Q entries based on IMU velocity variance
        # assume state = [dx, dy, dyaw], so match Q diag accordingly
        Q_dyn = np.zeros_like(self.Q)
        Q_dyn[0,0] = var_vx_imu * dt**2    # x process noise from vx variance
        Q_dyn[1,1] = var_vy_imu * dt**2    # y process noise from vy variance
        # yaw could also use imu angular rate covariance if available
        # try:
        #     cov_gyro = np.array(imu_now.angular_velocity_covariance).reshape(3,3)
        #     var_yawrate = cov_gyro[2,2]
        # except Exception:
        # var_yawrate = (0.001)**2  # fallback (rad/s)^2
        Q_dyn[2,2] = 0.0

        # propagate covariance
        self.P = self.P + (self.Q + Q_dyn) * dt

        # ---------------- Measurement ----------------
        lidar_now = self.get_interpolated_msg(self.lidar_buffer, t_now)
        if lidar_now is None:
            self.t_last = t_now
            return

        # Compute error measurement (local LiDAR error)
        x_imu = imu_now.pose.pose.position.x
        y_imu = imu_now.pose.pose.position.y
        _, _, yaw_imu = euler_from_quaternion([imu_now.pose.pose.orientation.x,
                                            imu_now.pose.pose.orientation.y,
                                            imu_now.pose.pose.orientation.z,
                                            imu_now.pose.pose.orientation.w])

        x_lidar = lidar_now.pose.pose.position.x
        y_lidar = lidar_now.pose.pose.position.y
        _, _, yaw_lidar = euler_from_quaternion([lidar_now.pose.pose.orientation.x,
                                                lidar_now.pose.pose.orientation.y,
                                                lidar_now.pose.pose.orientation.z,
                                                lidar_now.pose.pose.orientation.w])

        z = np.array([
            x_lidar - x_imu,
            y_lidar - y_imu,
            self.angle_diff(yaw_lidar, yaw_imu)
        ])

        # Measurement covariance from LiDAR
        cov = np.array(lidar_now.pose.covariance).reshape(6,6)
        R = np.zeros((3,3))
        R[0,0] = cov[0,0]
        R[1,1] = cov[1,1]
        R[2,2] = cov[5,5]

        # ---------------- EKF Update ----------------
        K = self.P @ np.linalg.inv(self.P + R)
        self.state = self.state + K @ (z - self.state)
        self.P = (np.eye(3) - K) @ self.P

        # ---------------- Correct IMU Odometry ----------------
        alpha = 0.1  # LiDAR contributes 20% of the error
        corrected_x = x_imu + alpha * self.state[0]
        corrected_y = y_imu + alpha * self.state[1]
        corrected_yaw = yaw_imu + alpha * self.state[2]

        corrected_msg = Odometry()
        corrected_msg.header.stamp = lidar_now.header.stamp
        corrected_msg.header.frame_id = lidar_now.header.frame_id
        corrected_msg.child_frame_id = lidar_now.child_frame_id
        corrected_msg.pose.pose.position.x = corrected_x
        corrected_msg.pose.pose.position.y = corrected_y
        corrected_msg.pose.pose.position.z = imu_now.pose.pose.position.z

        q_corr = quaternion_from_euler(0, 0, corrected_yaw)
        corrected_msg.pose.pose.orientation.x = q_corr[0]
        corrected_msg.pose.pose.orientation.y = q_corr[1]
        corrected_msg.pose.pose.orientation.z = q_corr[2]
        corrected_msg.pose.pose.orientation.w = q_corr[3]

        corrected_msg.pose.covariance = lidar_now.pose.covariance
        self.pub.publish(corrected_msg)

        # ---------------- Publish TF ----------------
        t = TransformStamped()
        t.header.stamp = lidar_now.header.stamp
        t.header.frame_id = corrected_msg.header.frame_id
        t.child_frame_id = corrected_msg.child_frame_id
        t.transform.translation.x = corrected_x
        t.transform.translation.y = corrected_y
        t.transform.translation.z = corrected_msg.pose.pose.position.z
        t.transform.rotation.x = q_corr[0]
        t.transform.rotation.y = q_corr[1]
        t.transform.rotation.z = q_corr[2]
        t.transform.rotation.w = q_corr[3]
        self.tf_broadcaster.sendTransform(t)

        self.t_last = t_now
   

def main(args=None):
    rclpy.init(args=args)
    node = ErrorStateEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
