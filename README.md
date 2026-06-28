
# UAV Control - Precision Landing (Hassas İniş) Project

This project aims to achieve offboard control of an Unmanned Aerial Vehicle (UAV) and enable autonomous "Precision Landing" using camera-based image processing within the Gazebo (GZ) simulation environment, leveraging the ROS 2 (Robot Operating System 2) and PX4 Autopilot infrastructure. The project integrates image processing, LiDAR mapping, and semi-autonomous remote control within an "all-in-one drone" framework. It is released as open source to support academic research.

## Tools Used
- ros_gz_bridge
- tf2
- rviz 
- opencv
- slam_toolbox 

## Setup
```bash
cd ~/your_ros_workspace/src
git clone https://github.com/semihberat/ROS2-PX4-Precision-Landing-and-Gamepad-Control.git --recursive
cd ~/your_ros_workspace # or cd ..
colcon build 
source install/local_setup.bash
```

There is a `models` folder inside the `src` directory within the workspace.
Move the `x500_all_sensors` folder—located inside the `models` folder—to `PX4_Autopilot/Tools/simulation/gz/models`.

## Çalıştırma

```bash
cd ~/PX4-Autopilot # PX4 folder
PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=aruco PX4_SIM_MODEL=gz_x500_all_sensors ./build/px4_sitl_default/bin/px4 -i 0
```

**Terminal 2: Micro DDS A (Bridge)**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3: Joy Controller**
```bash
ros2 run joy joy_node
```

**Terminal 4: C++ Flight Controller Node**
```bash
ros2 run precision_landing flight_controller
```
*(This node handles autonomous precision landing PID operations and control input mixing).*

**Terminal 5: Camera Sensor Bridge (Gazebo -> ROS)**
```bash
ros2 run ros_gz_bridge parameter_bridge "/world/aruco/model/x500_all_sensors/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
```

**Terminal 6: Camera Aruco Detector Node**
```bash
ros2 run precision_landing camera.py
```

**Terminal 6: Simulation Clock (it's important for realtime mapping)**
```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```
**Terminal 7: Lidar Sensor**
```bash
ros2 run ros_gz_bridge parameter_bridge /world/aruco/model/x500_all_sensors/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -r /world/aruco/model/x500_all_sensors/link/link/sensor/lidar_2d_v2/scan:=/scan --ros-args -p use_sim_time:=true
```

**Terminal 8: Start Cartographer**
```bash
ros2 launch precision_landing online_async_launch.py use_sim_time:=true
```

**Terminal 9: Start RVIZ**
```bash
ros2 launch precision_landing visualize.launch.py
```

*(When the ArUco marker is detected, its center coordinates are calculated and sent to the controller).*
