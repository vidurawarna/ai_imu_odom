import rclpy
from rclpy import logging
from sensor_msgs.msg import Imu
from ament_index_python.packages import get_package_share_directory
from collections import deque
import transforms3d
import os
import copy
from .inference import *

class AIODOM:
    def __init__(self, model_pt,  callback, calibrate_heading=False, calibration_distance=1.0):
        '''
            AIODOM collects gyroscope, linear accelaration, orientations readings by subscribing to an IMU topic (topic frequency should be 200Hz)
            Subscriber keeps a window of size 200 to collect the data
            Then, each new 10 readings will be taken inside the window and remove the first 10 readings in the window, then feed to the model
            After every 10 new samples, we get an estiation of the new position of the imu
            after every estimation, the estimated position and the pose of the imu is published for visualization
            
            callback: Callback function which accepts 3 tuples:      position, orientation, velocity
                                                                    [x, y, z], [qw, qx, qy, qz], [vx, vy, vz]
        '''
        pkg_dir = get_package_share_directory("ai_odom_3d_ros2")
        self.model_path = os.path.join(pkg_dir, 'models', model_pt)
        
        self.callback = callback
        self.imu_data = deque(maxlen=200)   #[time, qw, qx, qy, qz, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z]
        
        self.logger = logging.get_logger("AIODOM")
        self.logger.info("Make sure IMU rate is 200Hz and reference frame is ENU")
        self.logger.warn("If using VectorNav IMU on ROS2, make sure 'use_enu' parameter in vectornav driver is set to false")
        
        self.logger.info("Using Model: %s"%self.model_path)
        
        # State flags
        self.bias_calibrated = False # Gyro bias calibration
        self.rp_calibrated = False   # Roll Pitch calibration
        self.heading_calibrated = False  # Pedestrian yaw and path heading Calibration
        
        if not calibrate_heading:
            self.heading_calibrated = True
        self.calibration_distance = calibration_distance
        
        # Calibration matrices
        self.roll_pitch_CRMat = np.identity(3)      # Roll-pitch correction rotation matrix
        self.heading_CRMat = np.identity(3)         # Pedestrian heading correction rotation matrix
        self.path_heading_CRMat = np.identity(3)    # Path heading correction rotation matrix
        
        self.estimator = ResNet(self.model_path)
        
        self.output_buffer = []
        
    def process(self, imu_reading: Imu):
        '''
            Processes IMU readings and tracks the position of a pedestrian
            
            imu_reading : (sensor_msgs/Imu) IMU Message
        '''
        # Get IMU Input buffer
        data = self.input_buffer(imu_reading)
        
        # Check calibration status
        if not (self.is_bias_calibrated()):
            return
        if not (self.is_roll_pitch_calibrated()):
            return
        
        if (len(data) >= 200):
            imu_data = np.array(data)
            p_xyz, v_xyz, cov_vxvyvz = self.estimator.get_estimate(imu_data[:200, 1:5], imu_data[:200, 5:8], imu_data[:200, 8:11], imu_data[-1][0])
            position = (float(p_xyz[0]), float(p_xyz[1]), float(p_xyz[2]))
            velocity = (float(v_xyz[0]), float(v_xyz[1]), float(v_xyz[2]))
            cov = (float(cov_vxvyvz[0]), float(cov_vxvyvz[1]), float(cov_vxvyvz[2]))

            # Roll pitch correction
            rMat = transforms3d.quaternions.quat2mat(imu_data[-1][1:5])
            orientation = transforms3d.quaternions.mat2quat(rMat.dot(self.roll_pitch_CRMat))
            
            # Adjust heading
            success, position, orientation, velocity = self.adjust_heading(position, orientation, velocity)
            if success:
                # First send out buffered output, if any
                if (len(self.output_buffer) > 0):
                    for output in self.output_buffer:
                        _, p, o, v = self.adjust_heading(output[1], output[2], output[3])
                        self.callback(output[0], p, o, v, output[4])
                    self.output_buffer.clear()
                        
                self.callback(data[-1][0], position, orientation, velocity, cov)
            else:
                # Buffer output for future adjustments
                self.output_buffer.append([copy.copy(data[-1][0]), 
                                           copy.copy(position), 
                                           copy.copy(orientation), 
                                           copy.copy(velocity),
                                           copy.copy(cov)])
                
            # Remove earliest 10 IMU readings
            self.pop_input_buffer(10)
                
    def input_buffer(self, imu_reading: Imu):
        ''' Insert and update input data window

            :param imu_reading: IMU reading ROS msg
                                type: sensor_msgs/IMU
            :return IMU data window of size 200
                        Dimension: window_size x [time, 
                                                qw, qx, qy, qz,
                                                vel.x, vel.y, vel.z,
                                                acc.x, acc.y, acc.z]
        '''
        data = np.array([(imu_reading.header.stamp.sec +  imu_reading.header.stamp.nanosec * 1e-9),
                         imu_reading.orientation.w, imu_reading.orientation.x, imu_reading.orientation.y, imu_reading.orientation.z,
                         imu_reading.angular_velocity.x, imu_reading.angular_velocity.y, imu_reading.angular_velocity.z,
                         imu_reading.linear_acceleration.x, imu_reading.linear_acceleration.y, imu_reading.linear_acceleration.z])
        
        self.imu_data.append(data)
        return self.imu_data
    
    def pop_input_buffer(self, count: int):
        '''
            Removes the earliest 'count' number of items from the input buffer window
        '''
        for i in range(count):
            self.imu_data.popleft()
    
    def is_bias_calibrated(self) -> bool:
        '''
            Checks bias calibration status. If calibration is not complete, attempts calibration.
            
            Returns True if calibrated. False otherwise
        '''
        # Attempt calibration if not calibrated
        if not(self.bias_calibrated) and (len(self.imu_data) >= 100):
            # Average first 100 imu angular velocities for bias calculation
            self.estimator.set_gyro_bias(np.mean(np.array(self.imu_data)[:100, 5:8], axis=0),
                                         self.imu_data[-1][0])
            self.bias_calibrated = True
        
        return self.bias_calibrated
    
    def is_roll_pitch_calibrated(self) -> bool:
        '''
            Checks roll-pitch calibration status. If calibration is not complete, attempts calibration.
            
            Returns True if calibrated. False otherwise
        '''
        # Attempt calibration if not calibrated
        if not(self.rp_calibrated) and (len(self.imu_data) >= 200):
            a_x = 0.0
            a_y = 0.0
            a_z = 0.0
            for reading in self.imu_data:
                a_x += reading[8]
                a_y += reading[9]
                a_z += reading[10]
            
            roll = np.arctan2(-a_y, -a_z) 
            pitch = np.arctan2(-a_x, -np.sqrt(a_y*a_y + a_z*a_z))
            yaw = 0.0
            
            self.logger.info("Roll: " + str(round(roll * 180 / np.pi, 2)) + "\tPitch: " + str(round(pitch * 180 / np.pi, 2)))
            self.roll_pitch_CRMat = transforms3d.euler.euler2mat(roll, pitch, yaw).T
            self.rp_calibrated = True
        
        return self.bias_calibrated
    
    def adjust_heading(self, position=tuple, orientation=tuple, velocity=tuple) -> tuple:
        '''
            Corrects path heading and pedestrian heading
            If heading is not calibrated, attempts to calibrate the heading.
            
            Returns: success, position, orientation
                    
                    success: False if heading not calibrated, and calibration not feasible
                             position : x, y, z
                             orientation: qw, qx, qy, qz
        '''
        # Attempt calibration if not calibrated
        if not(self.heading_calibrated) and (np.hypot(position[0], position[1]) >= self.calibration_distance):
            path_heading = np.arctan2(position[1], position[0])
            
            yaw = 0.0
            for reading in self.imu_data:
                rMat = transforms3d.quaternions.quat2mat(reading[1:5])
                rp_corrected_orientation = transforms3d.quaternions.mat2quat(rMat.dot(self.roll_pitch_CRMat))
                yaw += transforms3d.euler.quat2euler(rp_corrected_orientation)[2]
            yaw /= len(self.imu_data)
            
            self.logger.info("Path Heading Correction: " + str(round(path_heading * 180 / np.pi, 2)) + "\tYaw: " + str(round(yaw * 180 / np.pi, 2)))
            self.path_heading_CRMat = transforms3d.euler.euler2mat(path_heading, 0.0, 0.0, 'szxy').T
            #-yaw was the change done for heading correction for dog odometry
            self.heading_CRMat = transforms3d.euler.euler2mat(-yaw, 0.0, 0.0, 'szxy').T
            self.heading_calibrated = True
            
        if (self.heading_calibrated):
            adjusted_position = (self.path_heading_CRMat @ np.array(position).reshape(3,1)).reshape(3,)
            adjusted_orientation = transforms3d.quaternions.mat2quat(transforms3d.quaternions.quat2mat(orientation) @ self.heading_CRMat)
            adjusted_velocity = (self.heading_CRMat @ np.array(velocity).reshape(3,1)).reshape(3,)
        
            return True, adjusted_position, adjusted_orientation, adjusted_velocity
        else:
            return False, position, orientation, velocity