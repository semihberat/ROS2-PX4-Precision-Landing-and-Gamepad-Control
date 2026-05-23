

```bash
cd ${px4-workspace} # (or `cd ~/PX4-autopilot`)
# start simulation
PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=aruco PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 0
MicroXRCEAgent udp4 -p 8888
ros2 run precision_landing flight_controller.cpp # 3rd terminal (start mission)
ros2 run joy joy_node # 4rd terminal (joy controller)

```

```bash
# Start camera bridge
ros2 run ros_gz_bridge parameter_bridge "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"

ros2 run precision_landing camera.py
# Start camera node
```
