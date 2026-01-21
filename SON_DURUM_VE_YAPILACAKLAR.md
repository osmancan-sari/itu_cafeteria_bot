# 🎯 BUGÜN YAPILACAKLAR LİSTESİ

Projenin bugün bitmesi için yapılması gereken her şey öncelik sırasına göre listelenmiştir.

---

## ✅ TAMAMLANAN İŞLER (Detaylı Açıklama)

### ✅ 1. `master.launch.py` Dosyasını Düzelt
**Dosya:** `cafeteria_robot_fsm/launch/master.launch.py`

**Yapılan Değişiklikler:**

#### 1.1 Dokümantasyon Güncellemesi
- **Satır 7:** "TurtleBot3 robot" → "Waiter robot" olarak değiştirildi
- Dokümantasyon artık doğru robotu tanımlıyor

#### 1.2 Package Path Düzeltmeleri
- **Eklendi:** `pkg_waiter_robot = get_package_share_directory('waiter_robot_description')`
- **Kaldırıldı:** TurtleBot3 paket referansları (zaten yoktu)
- **Korundu:** `pkg_nav2_bringup` (Nav2 navigation için gerekli)

#### 1.3 Gazebo Launch Düzeltmesi
- **Eski:** Basit IncludeLaunchDescription
- **Yeni:** OpaqueFunction ile gazebo_params.yaml desteği eklendi
- World dosyası ve gazebo params doğru şekilde yükleniyor

#### 1.4 Robot State Publisher Değişikliği
- **Eski:** TurtleBot3 URDF dosyasını direkt load ediyordu
- **Yeni:** `waiter_robot_description/launch/robot_state_publisher.launch.py` kullanıyor
- Doğru URDF (waiter_robot.xacro) artık yükleniyor
- `use_sim_time` ve `use_ros2_control` parametreleri doğru şekilde geçiliyor

#### 1.5 Robot Spawn Mekanizması
- **Eski:** TurtleBot3 spawn launch'ı kullanıyordu (çalışmıyordu)
- **Yeni:** Gazebo spawn_entity node kullanıyor
  - Topic: `robot_description`
  - Entity name: `waiter_robot`
  - Doğru şekilde Gazebo'da spawn ediyor

#### 1.6 Controller Spawner'lar Eklendi
- **`diff_drive_spawner`:** Differential drive controller'ı başlatır
  - Joint'leri: `left_back_wheel_joint`, `right_back_wheel_joint`
- **`joint_broad_spawner`:** Joint state broadcaster'ı başlatır
  - Tüm joint'lerin durumunu yayınlar

#### 1.7 Twist Mux Eklendi
- **Amaç:** Farklı kaynaklardan gelen cmd_vel komutlarını birleştirir
- **Input:** `/cmd_vel` (navigation'dan)
- **Output:** `/diff_cont/cmd_vel_unstamped` (controller'a)
- **Öncelik:** Navigation (10), Teleop (100) - şimdilik sadece navigation kullanılıyor

#### 1.8 Log Mesajları Güncellendi
- **Satır 197:** "Spawning TurtleBot3" → "Spawning Waiter Robot"
- Tüm log mesajları waiter_robot için uyarlandı

**Sonuç:** Artık `master.launch.py` waiter_robot'u doğru şekilde spawn ediyor ve tüm controller'ları başlatıyor.

---

### ✅ 2. `nav2_params.yaml` - base_frame_id Düzelt
**Dosya:** `cafeteria_simulation/config/nav2_params.yaml`

**Yapılan Değişiklik:**

#### 2.1 AMCL Parametresi Düzeltmesi
- **Satır 9:** `base_frame_id: "base_footprint"` → `base_frame_id: "base_link"`

**Açıklama:**
- waiter_robot URDF'inde `base_footprint` frame'i yok
- Robot `base_link` frame'ini kullanıyor
- AMCL (Adaptive Monte Carlo Localization) artık doğru frame'i kullanacak
- Nav2 localization düzgün çalışacak

**Etki Alanları:**
- AMCL particle filter doğru frame'de çalışacak
- Transform tree düzgün oluşacak
- Robot pozisyon tahmini doğru olacak

**Sonuç:** Nav2 localization artık waiter_robot ile uyumlu çalışacak.

#### 2.2 Nav2 Odom Topic Düzeltmesi (Yeni)
- **Satır 47:** `odom_topic: /odom` → `odom_topic: /diff_cont/odom`
- **Satır 347:** `odom_topic: "odom"` → `odom_topic: "/diff_cont/odom"`

**Açıklama:**
- Diff drive controller odometry’yi `/diff_cont/odom` yayınlıyor
- Nav2 varsayılan olarak `/odom` bekliyordu ve bu yüzden hareket üretmiyordu
- Odom topic’i Nav2’ye doğru şekilde bağlandı

**Sonuç:** Nav2 artık doğru odometry kaynağını kullanıyor.

---

### ✅ 3. `bumper.xacro` - Collision Name Hatası Düzelt
**Dosya:** `waiter_robot_description/urdf/bumper.xacro`

**Yapılan Değişiklik:**

#### 3.1 Collision Name Uyumsuzluğu Düzeltildi
- **Satır 37:** `<collision>bumper_link_collision</collision>` → `<collision>bumper_collision</collision>`

**Sorun:**
- Satır 6'da collision name `bumper_collision` olarak tanımlı
- Satır 37'de Gazebo contact sensor `bumper_link_collision` arıyordu
- Bu uyumsuzluk yüzünden bumper sensörü çalışmıyordu

**Çözüm:**
- Contact sensor artık doğru collision name'i (`bumper_collision`) kullanıyor
- Gazebo contact sensor düzgün çalışacak
- Bumper topic (`/bumper/contact`) doğru yayın yapacak

**Etki:**
- Bumper sensörü artık çalışır durumda
- Çarpışmalar algılanabilecek
- Collision detection node'unun işi kolaylaşacak

**Sonuç:** Bumper sensörü artık doğru çalışacak ve collision detection için kullanılabilir.

---

## 🔴 ÖNCELİK 1: DEVAM EDEN İŞLER (Kritik)

### ✅ 4. `README.md` Güncelle
**Dosya:** `README.md`

**Yapılan Değişiklikler:**

#### 4.1 Robot Tanımı Güncellemesi
- **Satır 3:** "TurtleBot3 robot" → "custom waiter robot" olarak değiştirildi
- README artık waiter_robot kullanıldığını açıkça belirtiyor

#### 4.2 Kurulum Talimatları Güncelleme
- **Satır 35:** TurtleBot3 paketleri kaldırıldı
- **Yeni:** Gerekli ROS2 paketleri listesi eklendi
  ```bash
  sudo apt install \
    ros-humble-nav2-msgs \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-twist-mux \
    ros-humble-slam-toolbox -y
  ```

#### 4.3 Build Komutu Güncelleme
- **Satır 42:** Build komutuna `waiter_robot_description` paketi eklendi
  ```bash
  colcon build --packages-select waiter_robot_description cafeteria_interfaces cafeteria_robot_fsm cafeteria_simulation
  ```

#### 4.4 Environment Variable Kaldırma
- **Satır 49:** `export TURTLEBOT3_MODEL=burger` satırı kaldırıldı
- waiter_robot için artık environment variable gerekmiyor

#### 4.5 Launch Komutu Güncelleme
- **Satır 50:** Launch komutu `master.launch.py` olarak güncellendi
  ```bash
  ros2 launch cafeteria_robot_fsm master.launch.py
  ```

**Sonuç:** README artık waiter_robot tabanlı sistemi doğru şekilde anlatıyor ve güncel komutları içeriyor.

#### 4.6 Docker Image Güncellemesi (Yeni)
- **Dosya:** `docker-compose.yml`
- **Eski Image:** `ros-gazebo:humble` (bulunamadığı için pull hatası veriyordu)
- **Yeni Image:** `osrf/ros:humble-desktop`

**Sonuç:** Docker image çekme hatası giderildi, container artık başarıyla açılıyor.

---

## 🟡 ÖNCELİK 2: ÖNEMLİ EKLEMELER

### ✅ 5. Temel Sensor Node'ları (FSR Yapay + Collision Node) - TAMAMLANDI

**Amaç:** FSM'in `/food_status` ve `/collision_alert` topic'lerini beklemesi nedeniyle FSR için yapay/senaryo tabanlı bir yayıncı, collision için gerçek bir köprü node'u eklendi. Böylece bumper temelli çarpışma algısı FSM'e doğru formatta iletiliyor.

#### 5.1 FSR Sensor Node (Yapay)
**Dosya:** `cafeteria_robot_fsm/cafeteria_robot_fsm/fsr_sensor_mock.py`

**Yapılanlar:**
- ROS2 node oluşturuldu: `fsr_sensor_mock`
- `/food_status` topic'ine FoodStatus publish ediyor
- 1 Hz timer ile sürekli `STATUS_PRESENT` gönderiyor
- Sabit weight değeri (0.5 kg) yayınlıyor

**Ekler:**
- `cafeteria_robot_fsm/setup.py` içine entry point eklendi:
  ```python
  'fsr_sensor_mock = cafeteria_robot_fsm.fsr_sensor_mock:main',
  ```

#### 5.2 Collision Detection Node (Gerçek)
**Dosya:** `cafeteria_robot_fsm/cafeteria_robot_fsm/collision_detection.py`

**Yapılanlar:**
- ROS2 node oluşturuldu: `collision_detection`
- `/bumper/contact` topic'ini dinliyor ve `/collision_alert` publish ediyor
- Spam önlemek için cooldown (0.5s) eklendi
- Opsiyonel IMU desteği eklendi (`enable_imu`, `imu_accel_threshold`)

**Ekler:**
- `cafeteria_robot_fsm/setup.py` içine entry point eklendi:
  ```python
  'collision_detection = cafeteria_robot_fsm.collision_detection:main',
  ```
- `cafeteria_robot_fsm/package.xml` içine `gazebo_msgs` dependency eklendi

#### 5.3 Launch Dosyasına Sensor Node'larını Ekle - TAMAMLANDI
**Dosya:** `cafeteria_robot_fsm/launch/master.launch.py`

**Yapılanlar:**
- `fsr_sensor_mock` node'u eklendi
- `collision_detection` node'u eklendi
- Node sırası FSM'den önce olacak şekilde ayarlandı

**Güncel Sıra:**
```
1. Gazebo
2. Robot State Publisher
3. Robot Spawn
4. Controllers
5. Twist Mux
6. Sensor Nodes (FSR Mock + Collision)
7. FSM Node (5 saniye gecikme)
```

**Sonuç:** FSM artık gerekli sensor topic'lerini alacak ve bumper tabanlı çarpışma akışı gerçek node ile sağlanacak. FSR tarafı gerçek sensör yerine **yapay/senaryo tabanlı** verilerle test edilecek.

---

### ✅ 7. Çalışma Ortamı ve Build Süreci Düzeltmeleri

**Yapılanlar:**
- Docker image pull hatası giderildi (image güncellendi)
- `ament_cmake` ve `xacro` eksikleri tespit edilip kurulum komutları dokümante edildi
- `colcon build` artık başarılı çalışıyor

**Notlar:**
- Container içinde her yeni terminalde şu iki komut zorunlu:
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/rosuser/itu_cafeteria_bot/install/setup.bash
  ```
- `ros2` komutları Windows PowerShell’de değil, container bash içinde çalıştırılmalı

---

### ✅ 6. Controller Parametreleri - Odom Yayını Düzelt
**Dosya:** `cafeteria_simulation/config/controller_params.yaml`

**Yapılan Değişiklikler:**
- `publish_odom: true`
- `odom_topic: odom`
- `enable_odom_tf: true`

**Açıklama:**
- Controller odom yayınlamıyordu; Nav2 bu yüzden hareket üretmiyordu
- Odom TF ve topic yayınları açıldı

**Sonuç:** Odom yayını etkinleşti ve Nav2 için gerekli altyapı tamamlandı.

---

## 🟢 ÖNCELİK 3: TEST VE DOĞRULAMA

### ⏳ 6. Sistem Testi
**Yapılacaklar:**

#### 6.1 Temel Başlatma Testi
- [ ] `master.launch.py` ile sistemi başlat
- [ ] Tüm node'ların başladığını kontrol et (`ros2 node list`)
- [ ] Topic'lerin yayın yaptığını kontrol et (`ros2 topic list`)

#### 6.2 Robot Spawn Doğrulama
- [ ] waiter_robot'un Gazebo'da göründüğünü doğrula
- [ ] Robot'un doğru pozisyonda spawn olduğunu kontrol et
- [ ] Robot'un fiziksel yapısının doğru göründüğünü kontrol et

#### 6.3 FSM Node Kontrolü
- [ ] FSM node'unun başladığını kontrol et
- [ ] FSM'in IDLE state'inde olduğunu doğrula (`ros2 topic echo /robot_state`)
- [ ] FSM'in operator_command service'ini sağladığını kontrol et

#### 6.4 Sensor Node Kontrolü
- [ ] FSR sensor node'unun `/food_status` yayınladığını kontrol et
- [ ] Collision detection node'unun `/bumper/contact` dinlediğini kontrol et
- [ ] Bumper'a dokunulduğunda `/collision_alert` yayınlandığını test et

#### 6.5 Operator Panel Testi
- [ ] Operator panel'i başlat (`ros2 run cafeteria_robot_fsm operator_panel`)
- [ ] `status` komutu ile robot durumunu görüntüle
- [ ] `deliver 1` komutu ile delivery başlat
- [ ] Robot'un delivery komutunu aldığını kontrol et

#### 6.6 Robot Hareket Testi (Nav2 olmadan)
- [ ] FSM'in `/cmd_vel` yayınladığını kontrol et
- [ ] Twist mux'un doğru yönlendirme yaptığını kontrol et
- [ ] Controller'ın cmd_vel'i aldığını kontrol et
- [ ] Robot'un hareket ettiğini gözlemle (eğer Nav2 yoksa manuel cmd_vel gönder)

**Öncelik:** Yüksek - Sistemin çalıştığından emin olmak için

---

### ⏳ 7. Nav2 Entegrasyonu (Eğer Map Varsa)
**Yapılacaklar:**

#### 7.1 Map Dosyası Kontrolü
- [ ] `cafeteria_simulation/worlds/` klasöründe map dosyaları var mı kontrol et
  - `map_save.yaml`
  - `map_save.pgm`
- [ ] Map dosyalarının geçerli olduğunu kontrol et

#### 7.2 Nav2 Launch'ı Aktif Etme
- [ ] `master.launch.py`'de Nav2 launch'ını uncomment et (satır 143-151)
- [ ] Map dosya yolunu doğru ayarla
- [ ] Localization launch'ını ekle (AMCL)
 - [ ] Initial pose gönder (AMCL için zorunlu olabilir)

#### 7.3 Navigation Testi
- [ ] Nav2 stack'in başladığını kontrol et
- [ ] Map'in yüklendiğini kontrol et (`ros2 topic echo /map`)
- [ ] Robot pozisyonunun doğru localize edildiğini kontrol et
- [ ] Navigation goal gönder ve robot'un hareket ettiğini test et

**Öncelik:** Orta - Navigation çalışmazsa robot sadece manuel kontrol edilebilir

---

## 📋 HIZLI REFERANS: DURUM TABLOSU

| Görev | Dosya | Durum | Öncelik |
|-------|-------|-------|---------|
| master.launch.py düzelt | `cafeteria_robot_fsm/launch/master.launch.py` | ✅ Tamamlandı | 🔴 Kritik |
| nav2_params.yaml düzelt | `cafeteria_simulation/config/nav2_params.yaml` | ✅ Tamamlandı | 🔴 Kritik |
| bumper.xacro düzelt | `waiter_robot_description/urdf/bumper.xacro` | ✅ Tamamlandı | 🔴 Kritik |
| README.md güncelle | `README.md` | ✅ Tamamlandı | 🟡 Önemli |
| FSR sensor node oluştur | `cafeteria_robot_fsm/cafeteria_robot_fsm/fsr_sensor_mock.py` | ✅ Tamamlandı | 🟡 Önemli |
| Collision detection node | `cafeteria_robot_fsm/cafeteria_robot_fsm/collision_detection.py` | ✅ Tamamlandı | 🟡 Önemli |
| Launch'a sensor ekle | `cafeteria_robot_fsm/launch/master.launch.py` | ✅ Tamamlandı | 🟡 Önemli |
| Controller odom ayarı | `cafeteria_simulation/config/controller_params.yaml` | ✅ Tamamlandı | 🔴 Kritik |
| Docker image düzeltme | `docker-compose.yml` | ✅ Tamamlandı | 🟡 Önemli |
| Sistem testi | - | ⏳ Bekliyor | 🟢 Test |

---

## ⏱️ TAHMİNİ SÜRE (Güncellenmiş)

- **✅ Öncelik 1 (Kritik - Tamamlandı):** ~1 saat ✅
- **⏳ Öncelik 2 (Önemli - Devam Ediyor):** ~1-2 saat
  - README güncelleme: ~15 dakika
  - FSR sensor node: ~30 dakika
  - Collision detection node: ~30 dakika
  - Launch entegrasyonu: ~15 dakika
- **⏳ Öncelik 3 (Test):** ~30-60 dakika

**Kalan Toplam:** ~2-3 saat

---

## 🚀 SONRAKI ADIMLAR (Öncelik Sırasına Göre)

1. **Sistem testi yap** (30-60 dk) - Her şeyin çalıştığından emin ol
2. **Operator panel testi** (10-15 dk) - Komutlar doğru çalışıyor mu
3. **FSM durum takibi** (10 dk) - `/robot_state` akışı doğru mu
4. **Nav2 entegrasyonu** (map + AMCL + initial pose) - Navigation doğrulaması
5. **Masa konumlarını kalibre et** - `TABLE_POSITIONS` gerçek map’e göre güncellenmeli
6. **Masa tanıma (ArUco / fiducial)** - Görsel marker ile doğrulama ve RECOVERY_SEARCH entegrasyonu
7. **FSR yapay senaryo publisher** - `STATUS_PRESENT/LIFTED/REMOVED` akışını test senaryolarıyla üret
8. **IMU entegrasyonu** - `collision_detection` node’una gerçek IMU verisi bağla
9. **Table moved / Recovery Search** - Marker yoksa dönme ve operatöre bildirim

---

## 💡 ÖNEMLİ NOTLAR

- **Yapay FSR yaklaşımı:** Gerçek FSR sensörü olmayacağı için `/food_status` verileri senaryo tabanlı publish edilecek.
- **Collision:** Collision algılama **yapılmalı**; bumper üzerinden köprü node ile FSM'e aktarım devam ediyor.
- **Nav2:** Eğer map yoksa, Nav2'yi şimdilik comment'te bırakabilirsiniz. FSM navigation olmadan da çalışır (sadece navigation yapamaz).
- **Test:** Her değişiklikten sonra sistemi test edin, böylece sorunları erken yakalarsınız.
- **Gazebo:** Robot spawn etmeden önce Gazebo'nun tamamen başlamasını bekleyin (5 saniye TimerAction var).

---

## ✅ BAŞARILI TAMAMLANMA KRİTERLERİ

- [x] `master.launch.py` waiter_robot spawn ediyor
- [x] Nav2 params waiter_robot ile uyumlu
- [x] Bumper sensörü doğru çalışıyor
- [ ] waiter_robot Gazebo'da görünüyor (test edilmeli)
- [ ] FSM node başlıyor (test edilmeli)
- [x] Sensor mockup node'ları eklendi (test edilmeli)
- [ ] Operator panel çalışıyor (test edilmeli)
- [ ] Temel sistem entegrasyonu tamamlanmış durumda

**İlerleme:** 🔴 Kritik ve 🟡 önemli kısımlar tamamlandı. Sırada testler var.

---

**Son güncelleme:** Bugün tamamlanan işler detaylıca açıklandı, bundan sonraki adımlar genişletildi.

---

## 📁 PROJEDEKİ TÜM KLASÖR/DOSYA AÇIKLAMALARI (DETAYLI)

Bu bölüm, projedeki tüm klasör ve dosyaların ne işe yaradığını açıklar. Aşağıdaki liste **tam ve güncel** yapı açıklamasıdır.

### Kök Dizin (Root)
- **`docker-compose.yml`**: Docker container ayarları (image, volume mount, DISPLAY).
- **`README.md`**: Kurulum ve çalıştırma adımları.
- **`BUGUN_YAPILACAKLAR.md`**: Bugün yapılan/ yapılacak tüm işler ve teknik özet.
- **`PROJE_YAPISI_DETAYLI_ANALIZ.md`**: Proje yapısının detaylı incelemesi.
- **`Robotics_Proposal (1) (1).pdf`**: Orijinal proje proposal dökümanı.
- **`.gitignore`**: Git ignore kuralları.
- **`build/`**: colcon build çıktıları (geçici).
- **`install/`**: colcon install çıktıları (run-time kullanılır).
- **`log/`**: colcon build ve ROS logları.
- **`frames_*.pdf / frames_*.gv`**: TF frame graph çıktıları (debug).

---

### 1) `cafeteria_interfaces/` — ROS2 Message/Service Tanımları
- **`CMakeLists.txt`**: Message/service üretimi için CMake config.
- **`package.xml`**: Paket meta ve bağımlılıklar.
- **`msg/`**:
  - `FoodStatus.msg`: Yemek durumu (FSR/yapay).
  - `CollisionAlert.msg`: Çarpışma uyarısı (bumper/IMU).
  - `RobotState.msg`: FSM durum yayın mesajı.
- **`srv/`**:
  - `OperatorCommand.srv`: Operator komut servisi.
  - `StartDelivery.srv`: Görev başlatma servisi.

---

### 2) `cafeteria_robot_fsm/` — FSM ve Operatör Arayüzü
- **`package.xml`**: ROS2 python paket bağımlılıkları.
- **`setup.py` / `setup.cfg`**: Python paket ve entry-point tanımı.
- **`resource/`**: ament index marker.
- **`launch/`**:
  - `master.launch.py`: Tüm sistemi başlatır (Gazebo + robot + controllers + FSM + sensor node’lar).
  - `fsm_only.launch.py`: Sadece FSM’i başlatır.
- **`cafeteria_robot_fsm/` (python modül)**:
  - `robot_state_machine.py`: Ana FSM logic.
  - `operator_panel.py`: CLI operator panel.
  - `fsr_sensor_mock.py`: Yapay FSR verisi yayınlar.
  - `collision_detection.py`: Bumper/IMU → CollisionAlert köprüsü.
  - `collision_detection_mock.py`: Eski mock (kullanım dışı).

---

### 3) `cafeteria_simulation/` — Gazebo + Nav2 + SLAM
- **`CMakeLists.txt`**: Simulation package build config.
- **`package.xml`**: Simulation bağımlılıkları (gazebo, nav2, control).
- **`launch/`**:
  - `simulation.launch.py`: Waiter robot’u Gazebo’da spawn eder.
  - `gazebo_world.launch.py`: Sadece world başlatır (eski).
  - `localization_launch.py`: AMCL + map_server.
  - `navigation_launch.py`: Nav2 navigation stack.
- **`config/`**:
  - `controller_params.yaml`: Diff drive controller parametreleri.
  - `gaz_ros2_ctl_use_sim.yaml`: ros2_control sim ayarları.
  - `gazebo_params.yaml`: Gazebo publish rate vb.
  - `mapper_params_online_async.yaml`: SLAM toolbox ayarları.
  - `nav2_params.yaml`: Nav2 parametreleri (odom_topic, costmap, planner).
  - `twist_mux.yaml`: cmd_vel birleştirme ayarları.
- **`worlds/`**:
  - `med_cafeteria.world`: Normal world.
  - `med_cafeteria_mapping.world`: Mapping için world.
  - `map_save.yaml/pgm`: Kaydedilmiş map dosyaları.
  - `*.data / *.posegraph`: SLAM serializasyon dosyaları.

---

### 4) `waiter_robot_description/` — Robot URDF/Xacro
- **`package.xml`**: Robot description package meta.
- **`setup.py` / `setup.cfg`**: Python package config.
- **`launch/`**:
  - `robot_state_publisher.launch.py`: URDF yayınlar.
  - `display.launch.py`: RViz ile görüntüleme.
- **`urdf/`**:
  - `waiter_robot.xacro`: Ana robot tanımı (include diğer xacro’lar).
  - `waiter_robot_core.xacro`: Gövde, tekerlek, tray vb.
  - `ros2_control.xacro`: ros2_control plugin.
  - `lidar.xacro`: LIDAR sensörü.
  - `camera.xacro`: Kamera sensörü.
  - `bumper.xacro`: Bumper + contact sensor.
  - `gazebo_control.xacro`: Eski diff_drive plugin (kullanılmıyor).
  - `inertial_macros.xacro`: Inertia macro’ları.
  - `waiter_robot_display.rviz / map_display.rviz`: RViz config’leri.
- **`waiter_robot_description/` (python modül)**:
  - `state_publisher.py`: Alternatif state publisher (genelde kullanılmıyor).

---

### 5) Diğer Önemli Dosyalar
- **`frames_*.pdf / frames_*.gv`**: TF frame graph debug çıktıları.
- **`build/`, `install/`, `log/`**: colcon build/install/log dizinleri.

