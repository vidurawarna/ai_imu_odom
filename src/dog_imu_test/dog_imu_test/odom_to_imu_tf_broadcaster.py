import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
import math
import transforms3d.quaternions as quat
import transforms3d

#--------------------------------------------------------------------------------------------------------
# This node is used to flip the odometry topic and odom tf by 180 degrees around x-axis
# This is done because the AI-IMU model trained for the go2 dog publishes flipped odometry (z- axis down)
#--------------------------------------------------------------------------------------------------------

class OdomToTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_imu_tf_broadcaster')
        
        self.subscription = self.create_subscription(
            Odometry,
            'ai_odom_raw',
            self.odom_callback,200)
        
        self.q_flip = quat.axangle2quat([1, 0, 0], np.pi)  # Quaternion for 180-degree rotation
        self.frame_id = 'odom'
        self.child_frame = 'imu_link'
        
        self.tf_odom_to_imu_broadcaster = TransformBroadcaster(self)
        self.odom_pub =  self.create_publisher(Odometry, 'ai_odom', 200)
    


    def odom_callback(self, msg):

        # Extract original quaternion
        q_orig = msg.pose.pose.orientation
        q_orig_array = np.array([q_orig.w, q_orig.x, q_orig.y, q_orig.z])  # wxyz format

        # Compute the new quaternion (q_new = q_flip * q_orig)
        q_new_array = quat.qmult(self.q_flip, q_orig_array)

        # publish transform 
        t = TransformStamped()
        
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = -msg.pose.pose.position.y
        t.transform.translation.z = -msg.pose.pose.position.z

        t.transform.rotation.w = q_new_array[0]
        t.transform.rotation.x = q_new_array[1]
        t.transform.rotation.y = q_new_array[2]
        t.transform.rotation.z = q_new_array[3]

        self.tf_odom_to_imu_broadcaster.sendTransform(t)


        # publish odom
        odom = Odometry()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame
        odom.header.stamp = msg.header.stamp
        
        odom.pose.pose.position.x = msg.pose.pose.position.x
        odom.pose.pose.position.y = -msg.pose.pose.position.y
        odom.pose.pose.position.z = -msg.pose.pose.position.z
        odom.pose.pose.orientation.w = q_new_array[0]
        odom.pose.pose.orientation.x = q_new_array[1]
        odom.pose.pose.orientation.y = q_new_array[2]
        odom.pose.pose.orientation.z = q_new_array[3]

        odom.twist.twist.linear.x = msg.twist.twist.linear.x
        odom.twist.twist.linear.y = -msg.twist.twist.linear.y

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# if __name__ == '__main__':
#     main()