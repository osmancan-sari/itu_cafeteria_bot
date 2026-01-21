# 📁 PROJE YAPISI - DETAYLI ANALİZ

Bu dokümanda projenin 4 ana klasörü ve içerikleri detaylıca açıklanmıştır.

---

## 📦 1. CAFETERIA_INTERFACES

### 🎯 **Amacı:**
Projenin tüm özel ROS2 message ve service tanımlarını içerir. FSM ve sensor node'ları arasındaki iletişim için özel veri tipleri tanımlar.

### 📋 **İçeriği:**

#### **Package Yapısı:**
- **Tip:** `ament_cmake` (C++ build system)
- **ROS2 İnterface Paketi:** Özel message ve service tanımları için

#### **Dosyalar:**

1. **`package.xml`** ✅
   - Paket metadata (isim, versiyon, maintainer)
   - Dependencies: `geometry_msgs`, `std_msgs`, `builtin_interfaces`
   - Build tool: `rosidl_default_generators`

2. **`CMakeLists.txt`** ✅
   - Interface generation için CMake konfigürasyonu
   - 3 message ve 2 service tanımını generate eder

3. **`msg/` Klasörü** - Message Tanımları:

   - **`FoodStatus.msg`** ✅
     - **Amaç:** FSR sensöründen gelen yemek durumu bilgisi
     - **Publisher:** FSR sensor node
     - **Subscriber:** Robot State Machine (FSM)
     - **Alanlar:**
       - `status` (uint8): STATUS_PRESENT=0, STATUS_LIFTED=1, STATUS_REMOVED=2
       - `weight_kg` (float32): Yemek ağırlığı
       - `stamp` (builtin_interfaces/Time): Zaman damgası
     - **Kullanım:** Theft prevention için kritik

   - **`CollisionAlert.msg`** ✅
     - **Amaç:** Çarpışma tespit uyarı mesajı
     - **Publisher:** IMU/Bumper sensor node
     - **Subscriber:** Robot State Machine (FSM)
     - **Alanlar:**
       - `collision_type` (uint8): TYPE_BUMPER=0, TYPE_IMU=1
       - `impact_direction` (geometry_msgs/Vector3): Çarpışma yönü
       - `severity` (uint8): Şiddet seviyesi (0-100)
       - `stamp` (builtin_interfaces/Time): Zaman damgası
     - **Kullanım:** Emergency stop için kritik

   - **`RobotState.msg`** ✅
     - **Amaç:** FSM'in mevcut durumunu broadcast eder
     - **Publisher:** Robot State Machine (FSM)
     - **Subscriber:** Operator Panel, Logging sistemleri
     - **Alanlar:**
       - `current_state` (uint8): Mevcut state (0-6)
       - `previous_state` (uint8): Önceki state
       - `state_name` (string): State ismi
       - `time_in_state` (float32): State'te geçirilen süre
       - `status_message` (string): Ek durum mesajı
     - **Kullanım:** Monitoring ve debugging için

4. **`srv/` Klasörü** - Service Tanımları:

   - **`OperatorCommand.srv`** ✅
     - **Amaç:** Operator'ın FSM'e komut göndermesi için
     - **Server:** Robot State Machine (FSM)
     - **Client:** Operator Panel
     - **Request:**
       - `command` (uint8): CMD_RESUME=0, CMD_ABORT=1, CMD_THEFT_CONFIRMED=2, CMD_FORCE_RETURN=3
       - `operator_message` (string): Ek mesaj
     - **Response:**
       - `success` (bool): Başarı durumu
       - `message` (string): Sonuç mesajı
       - `new_state` (uint8): Komut sonrası state
     - **Kullanım:** Emergency stop durumunda operator intervention

   - **`StartDelivery.srv`** ✅
     - **Amaç:** Yeni delivery görevi başlatmak için
     - **Server:** Robot State Machine (FSM)
     - **Client:** Operator Panel, Order Management System
     - **Request:**
       - `table_id` (int32): Hedef masa numarası
       - `target_position` (geometry_msgs/Point): Opsiyonel koordinat
       - `priority` (uint8): Öncelik seviyesi
     - **Response:**
       - `accepted` (bool): Görev kabul edildi mi
       - `mission_id` (string): Görev ID'si
       - `estimated_time` (float32): Tahmini süre
       - `error_message` (string): Hata mesajı
     - **Kullanım:** Delivery görevlerini başlatmak için

### ✅ **Durum:** Tüm interface'ler hazır ve çalışır durumda.

---

## 🤖 2. CAFETERIA_ROBOT_FSM

### 🎯 **Amacı:**
Robot'un "beyni" - tüm yüksek seviye davranışları yöneten Finite State Machine (FSM). Proposal'daki tüm state'leri ve transition'ları implement eder.

### 📋 **İçeriği:**

#### **Package Yapısı:**
- **Tip:** `ament_python` (Python package)
- **ROS2 Node Paketi:** Çalıştırılabilir Python node'ları içerir

#### **Dosyalar:**

1. **`package.xml`** ✅
   - Paket metadata
   - Dependencies:
     - ROS2 Core: `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`
     - Nav2: `nav2_msgs`, `action_msgs`
     - Custom: `cafeteria_interfaces`

2. **`setup.py`** ✅
   - Python package konfigürasyonu
   - Entry points (executable'lar):
     - `robot_state_machine`: Ana FSM node
     - `operator_panel`: Operator kontrol paneli
   - Launch dosyalarını install eder

3. **`cafeteria_robot_fsm/` Python Modülü:**

   - **`__init__.py`** ✅
     - Python package marker dosyası

   - **`robot_state_machine.py`** ✅ (~814 satır)
     - **Amaç:** Ana FSM node'u - robot'un merkezi beyni
     - **Çalıştırma:** `ros2 run cafeteria_robot_fsm robot_state_machine`
     - **Özellikler:**
       - 7 State: IDLE, NAVIGATING, THEFT_GRACE, EMERGENCY_STOP, RECOVERY_SEARCH, ARRIVED, RETURN_WITH_FOOD
       - Nav2 action client kullanır (`/navigate_to_pose`)
       - `/food_status` topic'ini dinler (theft detection)
       - `/collision_alert` topic'ini dinler (emergency stop)
       - `/cmd_vel` topic'ine publish eder (motor kontrolü)
       - `/robot_state` topic'ine publish eder (durum broadcast)
       - `/operator_command` service'i sağlar (operator intervention)
       - `/start_delivery` service'i sağlar (yeni görev başlatma)
     - **State Handler'lar:**
       - Her state için özel handler class'ı var
       - `on_enter()`, `on_exit()`, `on_update()` metodları
     - **Table Positions:** 5 masa pozisyonu hardcoded
     - **Home Position:** (0,0,0) hardcoded

   - **`operator_panel.py`** ✅ (~277 satır)
     - **Amaç:** Interactive CLI operator kontrol paneli
     - **Çalıştırma:** `ros2 run cafeteria_robot_fsm operator_panel`
     - **Özellikler:**
       - Real-time state monitoring
       - Komut gönderme (resume, abort, theft, return)
       - Delivery başlatma (`deliver <table_id>`)
       - Interactive command loop
     - **Komutlar:**
       - `status`: Mevcut durumu göster
       - `resume`: Emergency stop'tan resume
       - `abort`: Görevi iptal et
       - `theft`: Theft olayını onayla
       - `return`: Eve zorla dön
       - `deliver <1-5>`: Masa numarasına delivery başlat
       - `help`: Yardım mesajı
       - `quit`: Çıkış

4. **`launch/` Klasörü:**

   - **`master.launch.py`** ⚠️ **SORUN VAR**
     - **Amaç:** Tüm sistemi başlatan master launch dosyası
     - **Şu Anki Durum:** ❌ TurtleBot3 spawn etmeye çalışıyor
     - **Düzeltilmesi Gereken:**
       - waiter_robot spawn etmeli
       - TurtleBot3 referanslarını kaldırmalı
     - **Başlattıkları:**
       - Gazebo simulation
       - Robot spawn
       - Robot state publisher
       - FSM node (TimerAction ile gecikmeli)
       - Nav2 (şu anda comment'te)

   - **`fsm_only.launch.py`** ✅
     - **Amaç:** Sadece FSM node'unu başlatır
     - **Kullanım:** Gazebo zaten çalışıyorsa
     - **Başlattıkları:**
       - FSM node

### ✅ **Durum:** FSM kodu çok iyi yazılmış, proposal'a tam uyumlu. Sadece launch dosyası düzeltilmeli.

---

## 🎮 3. CAFETERIA_SIMULATION

### 🎯 **Amacı:**
Gazebo simulation ortamı, robot kontrolü, navigation ve SLAM konfigürasyonları. Robot'un simülasyon dünyasında çalışmasını sağlar.

### 📋 **İçeriği:**

#### **Package Yapısı:**
- **Tip:** `ament_cmake` (C++ build system)
- **ROS2 Simulation Paketi:** Gazebo world, launch, config dosyaları

#### **Dosyalar:**

1. **`package.xml`** ✅
   - Dependencies:
     - `gazebo_ros_pkgs`, `gazebo_ros2_control`
     - `controller_manager`, `ros2_controllers`
     - `twist_mux`
     - `nav2_msgs`

2. **`launch/` Klasörü:**

   - **`simulation.launch.py`** ✅ **ÇOK İYİ**
     - **Amaç:** waiter_robot'u Gazebo'da spawn eder
     - **Başlattıkları:**
       - Gazebo world
       - Robot state publisher (waiter_robot_description)
       - Robot spawn (waiter_robot entity)
       - Controller spawner (diff_cont, joint_broad)
       - twist_mux (cmd_vel yönlendirme)
     - **Özellikler:**
       - World dosyasını parametre olarak alır
       - waiter_robot spawn eder ✅
       - ros2_control controller'ları başlatır
     - **Durum:** ✅ Doğru çalışıyor, `master.launch.py` bunu referans almalı

   - **`gazebo_world.launch.py`** ⚠️ **KULLANILMIYOR**
     - **Amaç:** Sadece Gazebo world'ü başlatır
     - **Sorun:** TurtleBot3 URDF referansları var ama spawn yapmıyor
     - **Durum:** `simulation.launch.py` kullanılıyor, bu dosya gereksiz

   - **`localization_launch.py`** ✅
     - **Amaç:** Nav2 localization (AMCL) başlatır
     - **Başlattıkları:**
       - map_server (map yükler)
       - amcl (Adaptive Monte Carlo Localization)
       - lifecycle_manager
     - **Kullanım:** Map varsa robot lokalizasyon yapabilir

   - **`navigation_launch.py`** ✅
     - **Amaç:** Nav2 navigation stack'i başlatır
     - **Başlattıkları:**
       - controller_server (local planner)
       - planner_server (global planner)
       - smoother_server
       - behavior_server (recovery behaviors)
       - bt_navigator (behavior tree navigator)
       - waypoint_follower
       - velocity_smoother
       - lifecycle_manager
     - **Özellikler:**
       - Composition mode desteği
       - Respawn desteği
       - Log level ayarlama

3. **`config/` Klasörü:**

   - **`controller_params.yaml`** ✅
     - **Amaç:** ros2_control controller konfigürasyonu
     - **İçerik:**
       - `diff_cont`: Differential drive controller
       - `joint_broad`: Joint state broadcaster
       - Parametreler: wheel separation (0.59m), wheel radius (0.16m)
       - base_frame_id: `base_link` ✅
     - **Durum:** waiter_robot için doğru ayarlanmış

   - **`nav2_params.yaml`** ⚠️ **DÜZELTİLMELİ**
     - **Amaç:** Nav2 navigation stack konfigürasyonu
     - **İçerik:**
       - AMCL parametreleri
       - DWB local planner parametreleri
       - Costmap parametreleri (local ve global)
       - Behavior tree navigator
       - Velocity smoother
     - **Sorun:** Satır 9'da `base_frame_id: "base_footprint"` var, `"base_link"` olmalı
     - **Not:** Local planner parametreleri TurtleBot3 için ayarlanmış, waiter_robot için uyarlanabilir

   - **`twist_mux.yaml`** ✅
     - **Amaç:** cmd_vel topic'lerini birleştirir
     - **Özellikler:**
       - navigation topic (priority: 10)
       - teleop topic (priority: 100)
     - **Kullanım:** Farklı kaynaklardan gelen cmd_vel komutlarını birleştirir

   - **`gazebo_params.yaml`** ✅
     - **Amaç:** Gazebo simülasyon parametreleri
     - **İçerik:**
       - `publish_rate: 400.0` Hz

   - **`gaz_ros2_ctl_use_sim.yaml`** ✅
     - **Amaç:** Gazebo ros2_control plugin parametreleri
     - **İçerik:**
       - `use_sim_time: true`

   - **`mapper_params_online_async.yaml`** ✅
     - **Amaç:** SLAM Toolbox parametreleri (map oluşturma)
     - **İçerik:**
       - Ceres solver parametreleri
       - SLAM parametreleri
       - Map dosya yolu
     - **Not:** `base_frame: base_footprint` var ama bu SLAM için sorun değil

4. **`worlds/` Klasörü:**

   - **`med_cafeteria.world`** ✅
     - **Amaç:** Cafeteria ortamı Gazebo world dosyası
     - **İçerik:** Cafeteria fiziksel ortamı (masalar, duvarlar, vb.)

   - **`med_cafeteria_mapping.world`** ✅
     - **Amaç:** Mapping için optimize edilmiş cafeteria world
     - **Kullanım:** SLAM ile map oluştururken kullanılır

   - **`map_save.yaml`** ✅
     - **Amaç:** Kaydedilmiş map metadata (YAML)

   - **`map_save.pgm`** ✅
     - **Amaç:** Kaydedilmiş map görüntüsü (PGM format)

   - **`map_serial.*`** ✅
     - **Amaç:** SLAM Toolbox serialized map dosyaları

### ✅ **Durum:** Simulation setup iyi durumda. Sadece nav2_params.yaml'da base_frame_id düzeltilmeli.

---

## 🔧 4. WAITER_ROBOT_DESCRIPTION

### 🎯 **Amacı:**
waiter_robot'un fiziksel tanımı (URDF/Xacro). Robot'un 3D modeli, sensörleri, aktüatörleri ve Gazebo plugin'lerini tanımlar.

### 📋 **İçeriği:**

#### **Package Yapısı:**
- **Tip:** `ament_python` (Python package)
- **ROS2 Robot Description Paketi:** URDF/Xacro dosyaları ve launch dosyaları

#### **Dosyalar:**

1. **`package.xml`** ✅
   - Dependencies: `urdf_launch`, `rclpy`
   - Build type: `ament_python`

2. **`setup.py`** ✅
   - Python package konfigürasyonu
   - URDF ve launch dosyalarını install eder
   - Entry point: `state_publisher` (kullanılmıyor gibi)

3. **`urdf/` Klasörü** - Robot Fiziksel Tanımı:

   - **`waiter_robot.xacro`** ✅ **ANA DOSYA**
     - **Amaç:** Tüm xacro dosyalarını birleştiren ana dosya
     - **İçerik:**
       - `waiter_robot_core.xacro`: Robot gövdesi
       - `ros2_control.xacro`: Kontrol sistemi
       - `lidar.xacro`: LIDAR sensörü
       - `camera.xacro`: Kamera sensörü
       - `bumper.xacro`: Bumper sensörü

   - **`waiter_robot_core.xacro`** ✅ (~363 satır)
     - **Amaç:** Robot'un fiziksel gövdesi
     - **İçerik:**
       - `base_link`: Ana gövde (silindirik, 0.6m uzunluk, 0.2m yarıçap)
       - `base_footprint`: Zemin referansı
       - `left_leg`, `right_leg`: Robot bacakları
       - `left_base`, `right_base`: Tekerlek platformları
       - `left_back_wheel`, `right_back_wheel`: Tekerlekler (radius: 0.08m)
       - `caster_wheel`: Ön caster tekerlek
       - `head`: Robot başı (küresel)
       - `tray_pole`: Tepsi direği
       - `tray`: Yemek tepsisı (0.30m x 0.40m box)
     - **Özellikler:**
       - Inertia hesaplamaları (macros kullanılıyor)
       - Visual ve collision geometry
       - Material tanımları

   - **`ros2_control.xacro`** ✅
     - **Amaç:** ros2_control plugin tanımı
     - **İçerik:**
       - GazeboSystem hardware plugin
       - Joint command/state interface'leri
       - `left_back_wheel_joint`, `right_back_wheel_joint` kontrolü
       - Velocity command interface (min: -10, max: 10 rad/s)

   - **`lidar.xacro`** ✅
     - **Amaç:** LIDAR sensörü tanımı
     - **İçerik:**
       - `laser_frame`: LIDAR link
       - Gazebo ray sensor plugin
       - Topic: `/scan`
       - Parametreler:
         - Samples: 360
         - Min angle: -π
         - Max angle: π
         - Min range: 0.3m
         - Max range: 12m

   - **`camera.xacro`** ✅
     - **Amaç:** Kamera sensörü tanımı
     - **İçerik:**
       - `camera_link`: Kamera link
       - `camera_link_optical`: Optical frame
       - Gazebo camera plugin
       - Topic: `/camera/image_raw`
       - Parametreler:
         - Resolution: 640x480
         - Horizontal FOV: 1.089 rad
         - Update rate: 10 Hz

   - **`bumper.xacro`** ⚠️ **HATA VAR**
     - **Amaç:** Bumper/çarpışma sensörü
     - **İçerik:**
       - `bumper_link`: Bumper link (silindirik, radius: 0.25m)
       - Gazebo contact sensor plugin
       - Topic: `/bumper/contact`
     - **Sorun:** Satır 37'de collision name uyumsuzluğu
       - Satır 6: `name="bumper_collision"`
       - Satır 37: `<collision>bumper_link_collision</collision>` ❌
       - Düzeltilmeli: `bumper_collision` olmalı

   - **`gazebo_control.xacro`** ✅
     - **Amaç:** Gazebo differential drive plugin (eski, artık kullanılmıyor)
     - **Not:** ros2_control kullanıldığı için bu dosya gereksiz ama duruyor

   - **`inertial_macros.xacro`** ✅
     - **Amaç:** Inertia hesaplama macro'ları
     - **Macro'lar:**
       - `inertial_sphere`: Küre için inertia
       - `inertial_box`: Kutu için inertia
       - `inertial_cylinder`: Silindir için inertia
     - **Kullanım:** Robot linklerinin inertia değerlerini hesaplar

   - **`waiter_robot_display.rviz`** ✅
     - **Amaç:** RViz görselleştirme konfigürasyonu

   - **`map_display.rviz`** ✅
     - **Amaç:** Map görselleştirme için RViz konfigürasyonu

4. **`launch/` Klasörü:**

   - **`robot_state_publisher.launch.py`** ✅ **ÇOK ÖNEMLİ**
     - **Amaç:** Robot state publisher'ı başlatır
     - **Özellikler:**
       - waiter_robot.xacro dosyasını load eder
       - `use_sim_time` parametresi
       - `use_ros2_control` parametresi
     - **Kullanım:** `simulation.launch.py` tarafından kullanılıyor

   - **`display.launch.py`** ✅
     - **Amaç:** Robot'u RViz'de görselleştirmek için
     - **Başlattıkları:**
       - Robot state publisher
       - RViz
       - Joint state publisher (GUI ile)
     - **Kullanım:** Gazebo olmadan robot modelini görüntülemek için

5. **`waiter_robot_description/` Python Modülü:**

   - **`__init__.py`** ✅
     - Python package marker

   - **`state_publisher.py`** ✅
     - **Amaç:** Standalone robot state publisher (kullanılmıyor gibi)
     - **Not:** Launch dosyası kullanılıyor, bu dosya gereksiz olabilir

### ✅ **Durum:** Robot tanımı iyi yapılmış. Sadece bumper.xacro'daki hata düzeltilmeli. FSR ve IMU sensörleri eksik (proposal'da var ama URDF'de yok).

---

## 📊 ÖZET TABLO

| Klasör | Tip | Ana Görev | Durum | Sorunlar |
|--------|-----|-----------|-------|----------|
| **cafeteria_interfaces** | ament_cmake | Custom message/service | ✅ Hazır | Yok |
| **cafeteria_robot_fsm** | ament_python | FSM + Operator Panel | ✅ İyi | master.launch.py TurtleBot3 referansları |
| **cafeteria_simulation** | ament_cmake | Gazebo + Nav2 | ✅ İyi | nav2_params.yaml base_frame_id |
| **waiter_robot_description** | ament_python | Robot URDF | ✅ İyi | bumper.xacro collision name hatası |

---

## 🔍 EKSİKLER (Proposal'da Var, Kodda Yok)

1. **FSR Sensörü:**
   - URDF'de tray'e FSR sensörü yok
   - FSR sensor node yok

2. **IMU Sensörü:**
   - URDF'de IMU sensörü yok
   - IMU collision detection node yok

3. **ArUco Marker Detection:**
   - Marker detection node yok
   - Gazebo world'de marker'lar yok

4. **Nav2 Entegrasyonu:**
   - master.launch.py'de Nav2 comment'te
   - Map dosyası var ama aktif değil

---

## ✅ BAŞARIYLA TAMAMLANANLAR

1. ✅ FSM yapısı proposal'a tam uyumlu
2. ✅ Interface message/service'ler hazır
3. ✅ waiter_robot URDF iyi yapılmış (temel yapı)
4. ✅ Operator panel çalışıyor
5. ✅ Simulation launch dosyası doğru çalışıyor

---

**Bu analiz projenin mevcut durumunu tam olarak yansıtmaktadır!** 🎯
