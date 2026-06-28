
# UAV Control - Precision Landing (Hassas İniş) Projesi

Bu proje, ROS 2 (Robot Operating System 2) ve PX4 Otopilot altyapısını kullanarak, Gazebo (GZ) simülasyon ortamında İnsansız Hava Aracı'nın (İHA) offboard kontrolünü ve kameradan görüntü işleme desteğiyle otonom "Hassas İniş" (Precision Landing) yapabilmesini amaçlamaktadır. Proje görüntü işleme, lidar mapping ve yarı otonom kumanda kontrolünü bir **all in one drone** çerçevesi içerisinde işlemektedir. Proje akademik çalışmalara faydalı olması açısından açık kaynaklı olarak sunulmaktadır.

## Kullanılan Araçlar
- ros_gz_bridge
- tf2
- rviz 
- opencv
- slam_toolbox 

## kurulum
```bash
cd ~/your_ros_workspace/src
git clone https://github.com/semihberat/ROS2-PX4-Precision-Landing-and-Gamepad-Control.git --recursive
cd ~/your_ros_workspace # or cd ..
colcon build 
```

workspace içerisinde src dizini içerisinde bir models klasörü bulunmakta.
models klasörü içerisindeki `x500_all_sensors`
`PX4_Autopilot/Tools/simulation/gz/models` içerisine taşıyınız.

## Çalıştırma

```bash
cd ~/PX4-Autopilot # (PX4'ün kendi yüklü olduğu dizin)
PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=aruco PX4_SIM_MODEL=gz_x500_all_sensors ./build/px4_sitl_default/bin/px4 -i 0
```

**Terminal 2: Mikro DDS Ajanı (Köprü)**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3: Joy Kontrolcüsü**
```bash
ros2 run joy joy_node
```

**Terminal 4: C++ Uçuş Kontrolcüsü Node'u**
```bash
ros2 run precision_landing flight_controller
```
*(Bu node, otonom hassas iniş PID işlemlerini ve kumanda girdi miksajını sağlar).*

**Terminal 5: Kamera Sensörü Köprüsü (Gazebo -> ROS)**
```bash
ros2 run ros_gz_bridge parameter_bridge "/world/aruco/model/x500_all_sensors/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
```

**Terminal 6: Kamera Görüntü İşleme Node'u**
```bash
ros2 run precision_landing camera.py
```

**Terminal 6: Simülasyon Zamanlayıcısı**
```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```
**Terminal 7: Lidar Sensörü**
```bash
ros2 run ros_gz_bridge parameter_bridge /world/aruco/model/x500_all_sensors/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -r /world/aruco/model/x500_all_sensors/link/link/sensor/lidar_2d_v2/scan:=/scan --ros-args -p use_sim_time:=true
```

**Terminal 8: Cartographer'i Başlat**
```bash
ros2 launch precision_landing online_async_launch.py use_sim_time:=true
```

**Terminal 9: RVIZ'i Başlat**
```bash
ros2 launch precision_landing visualize.launch.py
```

*(ArUco işaretçisi algılandığında merkez koordinatları hesaplanır ve kontrolcüye gönderilir).*
