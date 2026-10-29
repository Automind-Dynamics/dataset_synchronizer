This codebase is currently setup to work with Husky ROS2 Bags containing Compressed RGB, Compressed Depth, PointCloud, IMU, Wheel Odometry, etc.
### Build

    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    git clone https://github.com/Automind-Dynamics/dataset_synchronizer.git
    cd ..
    colcon build --packages-select synced_sensor_pub husky_description husky_localization
    source install/setup.bash

### Usage
**Terminal 1 (Play the ROS2 Bag with "--clock" argument)**

    ros2 bag play garage_roads2 --clock

**Terminal 2 (Local EKF, Gives odom -->base_footprint  tf and all required static tf)**

    ros2 launch husky_localization husky_localization_local.launch.py

**Terminal 3 (Synchronizer Node)**

    ros2 launch synced_sensor_pub synced_sensors.launch.py
Outputs to be used:

    /synced/depth/camera_info
    /synced/depth/image_raw
    /synced/rgb/camera_info
    /synced/rgb/image_raw
    /synced/points

