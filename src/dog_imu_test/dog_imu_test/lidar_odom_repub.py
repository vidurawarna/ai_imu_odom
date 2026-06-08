import rclpy
from rclpy.node import Node
import tf_transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_multiply

class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odom_republisher')

        # Known static transforms (replace with your values)
        # Format: (x, y, z, qx, qy, qz, qw)

        # transforms: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: odom}, child_frame_id: camera_init, transform: {translation: {x: 0.0, y: 0.0, z: 0.0}, 
        # rotation: {x: 0.004318155084772587, y: 0.14349125604483803, z: -0.0006261026898267427, w: 0.9896419660517436}}}]
        self.T_odom_camera_init = (0.0, 0.0, 0.0, 0.004318155084772587, 0.14349125604483803, -0.0006261026898267427, 0.9896419660517436)

        # transforms: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: livox_frame}, child_frame_id: base_link, transform: {translation: {x: -0.25, y: 0.0, z: -0.1}, 
        # rotation: {x: -0.004318155084772587, y: -0.14349125604483803, z: -0.0006261026898267427, w: 0.9896419660517436}}}]

        self.T_livox_base = (-0.25, 0.0, -0.35, -0.004318155084772587, -0.14349125604483803, -0.0006261026898267427, 0.9896419660517436)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'lidar_odom', 10)
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber to your odometry
        self.create_subscription(Odometry, 'Odometry', self.odom_callback, 10)

    def odom_callback(self, msg):
        # Extract camera_init -> livox_frame from incoming odometry
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Compose transforms: odom->camera_init -> incoming -> livox->base
        tx1, ty1, tz1, q1x, q1y, q1z, q1w = self.T_odom_camera_init
        tx2, ty2, tz2, q2x, q2y, q2z, q2w = px, py, pz, qx, qy, qz, qw
        tx3, ty3, tz3, q3x, q3y, q3z, q3w = self.T_livox_base

        # Position composition (simple, ignoring rotation coupling for brevity)
        # You can replace this with proper tf_transformations for accuracy
        q12 = quaternion_multiply([q1x, q1y, q1z, q1w], [q2x, q2y, q2z, q2w])
        q_final = quaternion_multiply(q12, [q3x, q3y, q3z, q3w])
        t_final = (
            tx1 + tx2 + tx3,
            ty1 + ty2 + ty3,
            tz1 + tz2 + tz3
        )

        # Publish Odometry in odom frame
        odom_msg = Odometry()
        odom_msg = msg
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = t_final[0]
        odom_msg.pose.pose.position.y = t_final[1]
        odom_msg.pose.pose.position.z = t_final[2]
        odom_msg.pose.pose.orientation.x = q_final[0]
        odom_msg.pose.pose.orientation.y = q_final[1]
        odom_msg.pose.pose.orientation.z = q_final[2]
        odom_msg.pose.pose.orientation.w = q_final[3]
        self.odom_pub.publish(odom_msg)


def main():
    rclpy.init()
    node = OdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
