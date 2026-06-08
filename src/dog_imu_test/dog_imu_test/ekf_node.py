import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from math import sin, cos, atan2, pi
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from collections import deque
import time
from tf_transformations import quaternion_from_euler  # install tf-transformations

def wrap_angle(angle):
    # Wrap angle between [-pi, pi] to avoid yaw jumps
    return (angle + pi) % (2 * pi) - pi


class EKF_fusion:
    def __init__(self):
        # state: [x, y, vx, vy, theta]
        self.state = np.zeros((5, 1))
        self.P = np.eye(5) * 1.0  # initial covariance
        # Process noise covariance
        # self.Q = np.diag([0.05, 0.05, 0.2, 0.2, 0.01])
        self.Q = np.diag([0.001, 0.001, 0.001, 0.001, 0.001])


        self.last_timestamp = None

    def predict(self, dt):
        # State transition matrix F
        F = np.eye(5)
        F[0, 2] = dt  # x += vx*dt
        F[1, 3] = dt  # y += vy*dt

        self.state = F @ self.state
        self.state[4, 0] = wrap_angle(self.state[4, 0])

        self.P = F @ self.P @ F.T + self.Q

    def update(self, z, R, H, innovation_threshold=None):
        
        # EKF update step
        # z: measurement vector
        # R: measurement covariance
        # H: measurement matrix 
        
        z = np.reshape(z, (H.shape[0], 1))
        y = z - H @ self.state

        if H.shape[0] == 3:  # lidar update: [x,y,theta]
            y[2, 0] = wrap_angle(y[2, 0])
        elif H.shape[0] == 5:  # AI update includes theta
            y[4, 0] = wrap_angle(y[4, 0])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.state[4, 0] = wrap_angle(self.state[4, 0])
        self.P = (np.eye(len(self.state)) - K @ H) @ self.P

    def compute_mahalanobis(self, z, R, H):
        x = self.state
        z = np.reshape(z, (H.shape[0], 1))
        y = z - H @ x
        S = H @ self.P @ H.T + R
        d2 = float(y.T @ np.linalg.inv(S) @ y)
        return d2

class OdometryFusionNode(Node):
    def __init__(self):
        super().__init__('odometry_fusion_node')

        self.sub_lidar = self.create_subscription(Odometry, "lidar_odom", self.lidar_callback, 10)
        self.sub_ai = self.create_subscription(Odometry, "ai_odom", self.ai_callback, 10)
        self.pub_fused = self.create_publisher(Odometry, "odom_fused", 10)
        self.publisher_ = self.create_publisher(Odometry, 'corrected_lidar_pose', 10)
        self.tf_odom_to_imu_broadcaster = TransformBroadcaster(self)

        self.frame_id = "odom"
        self.child_frame = "base_footprint"

        self.timer_period = 1.0 / 20.0  
        self.timer = self.create_timer(self.timer_period, self.run_filter_step)

        self.ekf = EKF_fusion()

        self.last_time = None

        # Store latest measurements and covariance matrices
        self.lidar_measurement = None
        self.lidar_cov = None
        self.ai_measurement = None
        self.ai_cov = None

        self.last_good_pose = None
        self.T_drift1 = np.eye(3)
        self.T_drift2 = np.eye(3)
        self.lidar_cov_thresh = 1e-4
        self.state = 'NORMAL'
        self.lidar_corrected = False

        # Measurement matrix H for lidar [x,y,theta]
        self.H_lidar = np.zeros((3, 5))
        self.H_lidar[0, 0] = 1  # x
        self.H_lidar[1, 1] = 1  # y
        self.H_lidar[2, 4] = 1  # theta

        # Measurement matrix H for AI_IMU_ODOM [x,y,vx,vy,theta]
        self.H_ai = np.eye(5)

        # This is the threshold for a mahalanobis distance d^2
        # self.lidar_threshold = 9.21  # ~95% for 3 variable measurements (x,y,theta)
        self.lidar_threshold = 20

        # meters; tolerance between sources to negelect AI_IMU_ODOM
        self.ai_lidar_distance_thresh = 3.0#1.5

    def pub_lidar_corr(self,x,y,theta):

        fused_msg = Odometry()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = self.frame_id
        fused_msg.child_frame_id = self.child_frame

        # Set fused position
        fused_msg.pose.pose.position.x = x
        fused_msg.pose.pose.position.y = y
        fused_msg.pose.pose.position.z = 0.0

        # Set fused orientation from theta
        q = quaternion_from_euler(0, 0, theta)
        fused_msg.pose.pose.orientation.x = q[0]
        fused_msg.pose.pose.orientation.y = q[1]
        fused_msg.pose.pose.orientation.z = q[2]
        fused_msg.pose.pose.orientation.w = q[3]


        self.publisher_.publish(fused_msg)
        # self.get_logger().info(f'Publishing pose: x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f} rad')

    def pose_to_matrix(self, x, y, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [c, -s, x],
            [s,  c, y],
            [0,  0, 1]
        ])

    def matrix_to_pose(self, T):
        x, y = T[0, 2], T[1, 2]
        theta = np.arctan2(T[1, 0], T[0, 0])
        return x, y, theta
    
    def lidar_callback(self, msg):
        # Extract lidar measurement: [x, y, theta]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Orientation quaternion -> yaw
        q = msg.pose.pose.orientation
        theta = self.quat_to_yaw(q.x, q.y, q.z, q.w)



        # self.lidar_measurement = np.array([x, y, theta])

        # Extract covariance for position and orientation
        cov = msg.pose.covariance
        # Safety check: covariance length 36 expected
        if len(cov) == 36:
            cov_x = cov[0]
            cov_y = cov[7]
            cov_theta = cov[35]
        else:
            cov_x = cov_y = cov_theta = 0.01  # default small covariance

        if self.state == 'NORMAL':
            if cov_x > self.lidar_cov_thresh or cov_y > self.lidar_cov_thresh:
                self.get_logger().info('LIDAR DEVIATED')
                
                self.T_drift1 = self.pose_to_matrix(*self.last_good_pose)
                # self.T_drift1 = self.pose_to_matrix(self.last_good_pose[0], self.last_good_pose[1],0.0)
                

                self.state = 'DEVIATED'
            else:
                # T_odom = self.pose_to_matrix(x, y, theta)
                # T_corr = self.T_drift @ T_odom
                # x, y, theta = self.matrix_to_pose(T_corr)
                if np.allclose(self.T_drift1, np.eye(self.T_drift1.shape[0])):
                    # self.last_good_pose = (x, y, self.ekf.state[4,0])
                    if self.ai_measurement is not None:
                        self.last_good_pose = (x, y, self.ai_measurement[4])
                    else:
                        self.last_good_pose = (x, y, self.ekf.state[4,0])
                        
                else:
                    T_odom = self.pose_to_matrix(x, y, theta)
                    T_new = self.T_drift1 @ np.linalg.inv(self.T_drift2) @ T_odom
                    
                    x, y, theta = self.matrix_to_pose(T_new)
                    # self.last_good_pose = (x, y, self.ekf.state[4,0])
                    if self.ai_measurement is not None:
                        self.last_good_pose = (x, y, self.ai_measurement[4])
                    else:
                        self.last_good_pose = (x, y, self.ekf.state[4,0])

  

        elif self.state == 'DEVIATED':
            if cov_x < self.lidar_cov_thresh and cov_y < self.lidar_cov_thresh:
                self.get_logger().info('LIDAR recovered. Back to NORMAL')
                # self.lidar_transition_happend = True
                self.T_drift2 = self.pose_to_matrix(x, y, theta)
                self.state = 'NORMAL'

                T_odom = self.pose_to_matrix(x, y, theta)
                T_new = self.T_drift1 @ np.linalg.inv(self.T_drift2) @ T_odom
                x, y, theta = self.matrix_to_pose(T_new)
                # self.T_drift = np.eye(3)
                # T_odom = self.pose_to_matrix(x, y, theta)
                # T_corr = self.T_drift @ T_odom
                # x, y, theta = self.matrix_to_pose(T_corr)
                # self.last_good_pose = (x, y, theta)
            # else:
            #     T_odom = self.pose_to_matrix(x, y, theta)
            #     T_corr =  self.T_drift @ T_odom
            #     x, y, theta = self.matrix_to_pose(T_corr)
                

        self.lidar_measurement = np.array([x, y, theta])
        # self.get_logger().warn(f"{self.T_drift}")
        
        self.lidar_cov = np.diag([cov_x, cov_y, cov_theta])
        self.pub_lidar_corr(x,y,theta)

    def ai_callback(self, msg):
        # Extract AI measurement: [x, y, vx, vy, theta]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        q = msg.pose.pose.orientation
        theta = self.quat_to_yaw(q.x, q.y, q.z, q.w)

        self.ai_measurement = np.array([x, y, vx, vy, theta])

        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance

        if len(pose_cov) == 36 and len(twist_cov) == 36:
            cov_x = pose_cov[0]
            cov_y = pose_cov[7]
            cov_theta = pose_cov[35]
            cov_vx = twist_cov[0]
            cov_vy = twist_cov[7]
        else:
            cov_x = cov_y = cov_theta = cov_vx = cov_vy = 0.01
        self.ai_cov = np.diag([cov_x, cov_y, cov_vx, cov_vy, cov_theta])


    def run_filter_step(self):
        
        dt = self.timer_period

        self.ekf.predict(dt)

    

        # ---- LIDAR update ----
        lidar_used = False
        if self.lidar_measurement is not None:

            d2 = self.ekf.compute_mahalanobis(
                self.lidar_measurement,
                self.lidar_cov,
                self.H_lidar
            )

            # if d2 < self.lidar_threshold:
            if self.state=='NORMAL':
                # if d2 < self.lidar_threshold:
                self.ekf.update(
                    self.lidar_measurement,
                    self.lidar_cov,
                    self.H_lidar
                )
                lidar_used = True
                # self.lidar_corrected = False
            else:
                pass
                # if self.lidar_transition_happend:
                #     self.lidar_transition_happend = False
                    # self.ekf.update(
                    #     self.lidar_measurement,
                    #     self.lidar_cov,
                    #     self.H_lidar
                    # )
                    # lidar_used = True
                # self.get_logger().warn(f"LIDAR innovation too high: {d2:.2f}")
                # self.get_logger().warn(f"LIDAR odom deviated")

                

        # ---- AI update only if LIDAR unused or consistent ----
        if self.ai_measurement is not None:
            
            if not lidar_used:
                # No lidar update apply AI anyway
                # self.ekf.update(
                #         self.ai_measurement,
                #         self.ai_cov,
                #         self.H_ai
                #     )
                d2 = self.ekf.compute_mahalanobis(
                    self.ai_measurement,
                    self.ai_cov,
                    self.H_ai
                )
                if d2 < 9.21:
                    self.ekf.update(
                        self.ai_measurement,
                        self.ai_cov,
                        self.H_ai
                    )
                else:
                    self.get_logger().warn(f"AI ODOM innovation too high: {d2:.2f}")

            else:
                # Compare AI vs LIDAR
                ai_xy = self.ai_measurement[:2]
                lidar_xy = self.lidar_measurement[:2]
                # lidar_xy = np.array([self.ekf.state[0,0], self.ekf.state[1,0]])
                dist = np.linalg.norm(ai_xy - lidar_xy)          

                if dist < self.ai_lidar_distance_thresh:
                    self.ekf.update(
                        self.ai_measurement,
                        self.ai_cov,
                        self.H_ai
                    )
                else:
                    self.get_logger().info(f"Skipping AI update (distance = {dist:.2f} m)")

        self.publish_fused_odom()


    def publish_fused_odom(self):
        fused_msg = Odometry()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = self.frame_id
        fused_msg.child_frame_id = self.child_frame

        # Set fused position
        fused_msg.pose.pose.position.x = self.ekf.state[0, 0]
        fused_msg.pose.pose.position.y = self.ekf.state[1, 0]
        fused_msg.pose.pose.position.z = 0.0

        # Set fused orientation from theta
        qx, qy, qz, qw = self.yaw_to_quat(self.ekf.state[4, 0])
        fused_msg.pose.pose.orientation.x = qx
        fused_msg.pose.pose.orientation.y = qy
        fused_msg.pose.pose.orientation.z = qz
        fused_msg.pose.pose.orientation.w = qw

        # Set fused linear velocity
        fused_msg.twist.twist.linear.x = self.ekf.state[2, 0]
        fused_msg.twist.twist.linear.y = self.ekf.state[3, 0]
        fused_msg.twist.twist.linear.z = 0.0


        cov = np.zeros((6, 6))
        cov[0, 0] = self.ekf.P[0, 0]  # x
        cov[1, 1] = self.ekf.P[1, 1]  # y
        cov[5, 5] = self.ekf.P[4, 4]  # theta (yaw)
        # velocities covariance in twist covariance
        twist_cov = np.zeros((6, 6))
        twist_cov[0, 0] = self.ekf.P[2, 2]  # vx
        twist_cov[1, 1] = self.ekf.P[3, 3]  # vy

        fused_msg.pose.covariance = tuple(cov.flatten())
        fused_msg.twist.covariance = tuple(twist_cov.flatten())

        self.pub_fused.publish(fused_msg)

        # publish transform 
        t = TransformStamped()
        
        t.header.stamp = fused_msg.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame

        t.transform.translation.x = self.ekf.state[0, 0]
        t.transform.translation.y = self.ekf.state[1, 0]
        t.transform.translation.z = 0.0

        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz

        self.tf_odom_to_imu_broadcaster.sendTransform(t)

    @staticmethod
    def quat_to_yaw(x, y, z, w):
        euler = tf_transformations.euler_from_quaternion([x, y, z, w])
        return euler[2]  # yaw

    @staticmethod
    def yaw_to_quat(yaw):
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        return q[0], q[1], q[2], q[3]


def main(args=None):
    rclpy.init(args=args)
    node = OdometryFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
