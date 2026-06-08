import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu 
import numpy as np
import traceback
from .process import AIODOM

class RobotLocalizer(Node):
    def __init__(self):
        super().__init__('ai_aided_localization')
        self.get_logger().info("Starting AI-aided IMU for robot localization!")
      
        self.declare_parameter('calibrate_heading',True)
        self.declare_parameter('calibration_distance',1.0)
        self.declare_parameter('reference_frame_id', "map")
        self.declare_parameter('frame_id', "human")
        self.declare_parameter('publish_path', False)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('imu_topic',"/vectornav/imu")
        self.declare_parameter('model_pt',"checkpoint_150_new.pt")

        calibrate_heading = self.get_parameter('calibrate_heading').get_parameter_value().bool_value 
        calibration_distance = self.get_parameter('calibration_distance').get_parameter_value().double_value 
        self.reference_frame_id = self.get_parameter('reference_frame_id').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value 
        self.path_enable = self.get_parameter('publish_path').get_parameter_value().bool_value
        self.tf_enable = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.model_pt = self.get_parameter('model_pt').get_parameter_value().string_value
        
        self.INS = AIODOM(self.model_pt, self.publish_odom, calibrate_heading, calibration_distance)
        
        self.odom_pub =  self.create_publisher(Odometry, 'ai_odom_raw', 200)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.path_enable:
            self.path_pub = self.create_publisher(Path, 'path', 200)
            self.path = Path()   
        self.subscription = self.create_subscription(Imu, self.imu_topic, self.callback, 200) 
        
        self.get_logger().info("Subscribed to: " + self.imu_topic)
        self.get_logger().info("Calibrate Heading: " + str(calibrate_heading))
        self.get_logger().info("Calibration Distance: " + str(calibration_distance))
        self.get_logger().info("Publish path: " + str(self.path_enable))

    def callback(self, imu_msg): 
        self.INS.process(imu_msg)
        # self.publish_odom(position=x, orientation=q, velocity=v)
        
    def publish_odom(self, time: float, position: tuple, orientation: tuple, velocity: tuple, cov: tuple):
        ''' Publish Odometry Message
            :param time: (float) Timestamp in seconds
            :param position: (Tuple) x, y, z
            :param orientation: (Tuple) w, x, y, z
            :param velocity: (Tuple) vx, vy, vz
        '''
        odom = Odometry()
        odom.header.frame_id = self.reference_frame_id
        odom.child_frame_id = self.frame_id
        odom.header.stamp.sec = int(time)
        odom.header.stamp.nanosec = int((time%1) * 1e9)
        
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        odom.pose.pose.orientation.x = orientation[1]
        odom.pose.pose.orientation.y = orientation[2]
        odom.pose.pose.orientation.z = orientation[3]
        odom.pose.pose.orientation.w = orientation[0]
        odom.twist.twist.linear.x = velocity[0]
        odom.twist.twist.linear.y = velocity[1]
        odom.twist.twist.linear.z = velocity[2]

        # Pose covariance
        odom.pose.covariance[0]  = (0.05**2) * cov[0]   # x
        odom.pose.covariance[7]  = (0.05**2) * cov[1]   # y
        odom.pose.covariance[14] = (0.05**2) * cov[2]   # z

        # Twist covariance
        odom.twist.covariance[0]  = cov[0]   # vx
        odom.twist.covariance[7]  = cov[1]   # vy
        odom.twist.covariance[14] = cov[2]   # vz

        self.odom_pub.publish(odom)

        if(self.tf_enable):
            self.pub_tf(odom)

        if (self.path_enable):
            self.publish_path(odom)

    
    def publish_path(self, odometry: Odometry):
        '''
            Publish accumulated path using odometry msgs
        '''
        # Pose sampling (min distance between path points is 10cm)
        if len(self.path.poses) > 0:
            if np.hypot(self.path.poses[-1].pose.position.x - odometry.pose.pose.position.x,
                        self.path.poses[-1].pose.position.y - odometry.pose.pose.position.y) < 0.1:
                 return

        pose  = PoseStamped()
        self.path.header = odometry.header
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

    def pub_tf(self, msg: Odometry):

        t = TransformStamped()

        # Use exact timestamp from odometry
        t.header.stamp = self.get_clock().now().to_msg()

        # Use same frame ids from odometry message
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id

        # Position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Orientation
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)
    
def main(args=None):
    rclpy.init(args=args)
    node = RobotLocalizer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as ex:
        print(str(ex))
        traceback.print_tb()   
