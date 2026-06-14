
# UAV Control - Precision Landing (Hassas İniş) Prejesi

Bu proje, ROS 2 (Robot Operating System 2) ve PX4 Otopilot altyapısını kullanarak, Gazebo (GZ) simülasyon ortamında İnsansız Hava Aracı'nın (İHA) offboard kontrolünü ve kameradan görüntü işleme desteğiyle otonom "Hassas İniş" (Precision Landing) yapabilmesini amaçlamaktadır.

## Proje Dizini (`src/` Odaklı İnceleme)

Projede öne çıkan üç ana paket bulunmaktadır:

*   **`precision_landing`**: İHA'nın kontrol edildiği ana ROS 2 paketidir (PID hesaplamaları, otonom hareket ve görüntü işleme burada dönüyor).
*   **`custom_msgs`**: Görüntü işleme ve otopilot arasındaki iletişim için kendi oluşturduğumuz özel mesaj tipini (`CPoints.msg`) barındırır.
*   **`px4_msgs`**: PX4-ROS 2 köprüsü (MicroXRCEAgent) ile haberleşebilmek için zorunlu olan, PX4 mesaj tanımlamalarını içeren kütüphanedir.

---

## Projenin Amacı

Proje, Gazebo simülasyonunda aşağı bakan kamerası *(downward camera)* olan bir x500 drone modeli (gz_x500_mono_cam_down) kullanarak yerdeki bir ArUco işaretçisini (marker) bulmak, hizalanmak ve üzerine iniş yapmaktır. Ayrı olarak bir kumanda (Joystick) aracılığıyla Offboard kontrolde manuel hız komutları da verilebilir.

---

## Matematiksel Hesaplamalar ve Mantık (Mantığın Özeti)

Sistem iki temel dosyadan matematiksel kararlar almaktadır:

### 1- Görüntü İşleme: `camera.py` (Script)
*   **İşlem:** Gazebo simülasyonundan `/world/aruco/model/.../image` topic'i üzerinden alınan sanal kamera görüntüsünü OpenCV (`cv_bridge`) aracılığıyla okur. `cv2.aruco` kütüphanesi (DICT_4X4_100) yardımıyla yerdeki işaretçiyi tarar.
*   **Merkez Tespiti (ArUco Merkezi):** Bulunan karenin 4 köşesinin (x, y) değerleri alınarak diyagonallerinden ArUco'nun merkez koordinatı bulunur (`box_cx, box_cy`).
*   **Kamera Ekranın Merkezi:** Çözünürlüğe göre (Genişlik/2, Yükseklik/2) olacak şekilde belirlenir (`cx, cy`).
*   **Normalizasyon:** (0, çözünürlük) arasında değişen piksel değerleri PID üzerinde stabil olması adına 0.0 ile 2.0 değeri arasına sıkıştırılır *(değerler width/height'e bölünüp 2 ile çarpılır)*. Tüm bu noktalar `/aruco_center` topic'ine fırlatılır.

### 2- Kontrol ve PID Mekanizması: `flight_controller.cpp`
*   **Kameradan Alınan Veri:** `CPoints` tipinde `/aruco_center` üzerinden gelen normalize edilmiş (ArUco Merkezi ve Kamera Çerçevesi Merkezi) koordinatları dinler. 
*   **Hata Pili Payı (Error):** `error_x = box_cx - cx` ve `error_y = box_cy - cy` şeklinde piksel hizalama hatası hesaplanarak hedef (setpoint) merkeze oturtulmaya çalışılır.
*   **PID Güncellemeleri:**
    *   **İleri / Geri Yön (Drone X Ekseni):** Kameranın Y (dikey) eksenindeki hata, Drone'un X (ileri/geri) hız komutuna bağlanmıştır. Aruco merkezden aşağıdaysa geri gelmesi için `x = pid_y.update(msg->box_cy)` doğrudan eşitlenir.
    *   **Sağ / Sol Yön (Drone Y Ekseni):** Kameranın X (yatay) eksenindeki hata, Drone'un Y (sağ/sol) hız komutuna bağlanmıştır. Aruco sağdaysa sağa uçmak için `-` (eksi) ile çarpılıp tersinir işlem uygulanır: `y = -pid_x.update(msg->box_cx);`
*   **Birleştirme:** Varsa joystick'ten gelen hız komutlarıyla (`velocity_`), PID'den gelen hata düzeltme komutları toplanarak PX4'e trajectory_setpoint (*hız referansı*) olarak gönderilir: `vx = velocity_.x + x` şeklinde.
*   *Güvenlik:* Marker'a olan (hata vektörünün uzunluğu) mesafe `< 0.01` olduğu takdirde drone yavaş yavaş aşağı (-Z) indirilmeye (0.5f hıza) başlar. Ayrıca kumandada (örneğin sol analog alt sol, sağ analog alt sağ olursa) ARM edebilecek özel bir acil koşul bulunmaktadır.

---

## Kurulum ve Kullanım Talimatları

Projenin derlenebilmesi ve çalıştırılabilmesi için PX4 kurulumlarının (SITL simülasyonu, MicroDDS agent) tam olması gereklidir.

### 1- Derleme (Build)

UAV çalışma alanının (workspace) root klasöründe paketleri derleyin:

```bash
cd ~/uav_control
colcon build 
source install/setup.bash
```

### 2- Çalıştırma 

Ayrı ayrı terminaller (hepsinde gerekli durumlarda source işlemlerini yapmayı unutmayın) açıp aşağıdaki adımları sırayla takip edin:

**Terminal 1: PX4 Simülasyonunu Başlatın (Gazebo)**
```bash
cd ~/PX4-Autopilot # (PX4'ün kendi yüklü olduğu dizin)
PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=aruco PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 0
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
ros2 run ros_gz_bridge parameter_bridge "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
```

**Terminal 6: Kamera Görüntü İşleme Node'u**
```bash
ros2 run precision_landing camera.py
```
*(ArUco işaretçisi algılandığında merkez koordinatları hesaplanır ve kontrolcüye gönderilir).*
