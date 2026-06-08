# AI-aided IMU Odometry
This repository shares the ROS2 implementations that were done for the work "AI-Aided IMU Odometry for Robust Mapping with Quadruped Robots in Challenging Environments". </br>

## Requirements
* Ubuntu 22
* ROS2 Humble 
* python libraries mentioned in ```requirements.txt```


## Setup
* Install ```ROS2``` by followng the guidance in [ROS2 installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
* Create a workspace for the project and a ```/src``` folder inside the workspace.
* Open a terminal and navigate to ```<workspace_name>/src``` and copy the folders of this repo inside the src. 
* Navigate the terminal to ```<workspace_name>/src``` and run the following command to install the required python libraries using pip3. 
>     pip3 install -r requirements.txt
Notes:
> Check whether the SLAM toolbox is installed after installing ROS2. </br>
> There can be other dependencies based on the system that you are using. Install them if the python scripts give errors while execution. </br>

## Downloading required files
* Download the model pointers and bag files ```/models```, ```/bag_files``` from [here](https://drive.google.com/drive/folders/1me--EW3P8-wOnhDiLfMDO1nRCdVXScPV?usp=sharing)
* Copy downloaded ```/bag_files``` to ```<workspace_name>/bag_files/```
* Copy downloaded ```/models``` to ```<workspace_name>/src/ai_odom_3d_ros2/models/```

## Launching
* Build the packages by navigating the terminal to ```<workspace_name>/``` 
>     colcon build
* Source the workspace in every terminal you run the packages.
>     <path_to_workspace>/install/setup.bash
* To run the SLAM test for LiDAR odometry mapping vs AI-aided IMU odometry mapping:
>     ros2 launch dog_imu_test slam_test_launch.py

* To run the SLAM test for LiDAR odometry mapping vs Fused odometry mapping:
>     ros2 launch dog_imu_test odometry_fusion_launch.py

* Both the launch files accept the following arguments to specify the bag_file to playback. You can use the absolute path of the ROS2 bag file if you did not copy the bag files to the locations as specified in the previous steps. (Please do not launch the both tasks at once because it may conflict the ROS topics and the required task may not be completed properly.)

>     ros2 launch dog_imu_test odometry_fusion_launch.py bag_file:= <path_to_bag_file>
>     ros2 launch dog_imu_test slam_test_launch.py bag_file:= <path_to_bag_file>

## Parameters for ```ai_odom_3d_ros2```
Parameters are configured in ```<pkg_directory>/config/params.yaml``` file.
* **imu_topic** : Subscribed IMU topic. (Default: /vectornav/imu)
* **reference_frame_id**: Parent frame id in which odometry will be published
* **frame_id**: Child frame id of published odometry
* **calibrate_heading**: If set to true, calibrates initial pedestrian heading and path heading to align with reference frame X axis 
* **calibration_distance**: Heading calibration will be performed considering readings upto this walking distance. (Default: 1m)
* **model_pt**: The trained model weights file name

Note:
> The mapping task will start after approximately after 10s when you launch the above tests. </br>
> Wait for the mapping bag to complete because the SLAM may perform loop closing. </br>
> The performance for AI-aided IMU odometry will depend on the processing power of the machine you are using. if the scripts manage to utilize the GPU, the performance will be better. </br>
> The mapping results may slightly vary upon this execution limitations. </br>
> Users can modify the  ```ekf_node.py``` from the package ```dog_imu_test``` to tune the parameters of the filter to obtain better results. </br>


