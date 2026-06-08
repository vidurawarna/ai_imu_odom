import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
import math
import transforms3d.quaternions as quat
import transforms3d
from scipy.spatial.transform import Rotation as R

#--------------------------------------------------------------------------------------------------------
# This node is used to flip the odometry topic and odom tf by 180 degrees around x-axis
# This is done because the AI-IMU model trained for the go2 dog publishes flipped odometry (z- axis down)
#--------------------------------------------------------------------------------------------------------

class OdomFlipper(Node):
    def __init__(self):
        super().__init__('flip_odom')
        
        self.subscription = self.create_subscription(
            Odometry,
            'ai_odom_raw',
            self.odom_callback,200)
        
        self.q_flip = quat.axangle2quat([1, 0, 0], np.pi)  # Quaternion for 180-degree rotation
        self.frame_id = 'odom'
        # self.child_frame = 'imu_link'
        self.child_frame = 'base_footprint'

        # imu_link to base_footprint tf
        self.offset = np.array([-0.3, 0.0, 0.0])
        
        # self.tf_odom_to_imu_broadcaster = TransformBroadcaster(self)
        self.odom_pub =  self.create_publisher(Odometry, 'ai_odom', 200)
    


    def odom_callback(self, msg):

        # Extract original quaternion
        q_orig = msg.pose.pose.orientation
        q_orig_array = np.array([q_orig.w, q_orig.x, q_orig.y, q_orig.z])  # wxyz format

        # Compute the new quaternion (q_new = q_flip * q_orig)
        q_new_array = quat.qmult(self.q_flip, q_orig_array) #[w,x,y,z]

        #------------------------apply tf to get base fooprint---------------------------
        p_oi = np.array([
            msg.pose.pose.position.x,
            -msg.pose.pose.position.y,
            -msg.pose.pose.position.z
        ])
        q_oi = np.array([
            q_new_array[1],
            q_new_array[2],
            q_new_array[3],
            q_new_array[0]
        ])

        R_oi = R.from_quat(q_oi).as_matrix()
        p_ob = p_oi + R_oi.dot(self.offset)
        
        # publish odom
        odom = Odometry()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame
        odom.header.stamp = msg.header.stamp
        
        odom.pose.pose.position.x = float(p_ob[0])
        odom.pose.pose.position.y = float(p_ob[1])
        odom.pose.pose.position.z = float(p_ob[2])
        odom.pose.pose.orientation.w = q_new_array[0]
        odom.pose.pose.orientation.x = q_new_array[1]
        odom.pose.pose.orientation.y = q_new_array[2]
        odom.pose.pose.orientation.z = q_new_array[3]

        odom.twist.twist.linear.x = msg.twist.twist.linear.x
        odom.twist.twist.linear.y = -msg.twist.twist.linear.y

        odom.pose.covariance[0] = msg.pose.covariance[0]  #x
        odom.pose.covariance[7] = msg.pose.covariance[7]  #y
        odom.twist.covariance[0] = msg.twist.covariance[0] #vx
        odom.twist.covariance[7] = msg.twist.covariance[7] #vy

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFlipper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# if __name__ == '__main__':
#     main()