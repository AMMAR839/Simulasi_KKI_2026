# ASV KKI 2026 ROS 2 / GZ Sim Workspace

Workspace baru:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws`

Referensi lokal yang dipakai dan tidak diubah:

- `/home/ammar/Documents/asv_simulation/Sosialisasi KKI 2026 - ASV.pdf`
- `/home/ammar/Documents/asv_simulation/Savinah One 2025.pdf`
- `/home/ammar/Documents/asv_simulation/Final_TOT.3dm`
- `/home/ammar/Documents/asv_simulation/vrx`
- `/home/ammar/Documents/asv_simulation/SINGABOAT-VRX`
- `/home/ammar/Documents/asv_simulation/gz-sensors`
- `/home/ammar/Documents/asv_simulation/asv_wave_sim`

## Isi Workspace

- `src/asv_description`: URDF/Xacro dan SDF model kapal aktif berbasis VRX `roboboat02`, plus script konversi cadangan untuk `Final_TOT.3dm`.
- `src/asv_gazebo`: world lomba 30 x 30 m, 10 pasang buoy merah-hijau, obstacle, target imaging, dock, dan docking buoy biru.
- `src/asv_sensors`: konfigurasi sensor dan bridge GZ-ROS.
- `src/asv_control`: joystick mode manager, manual/autonomous mux, dan konversi `/cmd_vel` ke thrust kiri-kanan plus sudut servo thruster.
- `src/asv_navigation`: waypoint follower GPS/compass/LiDAR dan konfigurasi `robot_localization`.
- `src/asv_bringup`: launch utama.

## Ringkasan Sistem Kontrol

Workspace ini punya dua mode kontrol:

- Manual mode: kapal dikendalikan joystick/gamepad.
- Autonomous mode: kapal mengikuti waypoint lintasan secara otomatis.

Startup mode dikontrol dari argumen launch `auto_mode`:

- `auto_mode:=false`: kapal start di manual mode. Joystick/gamepad otomatis dijalankan.
- `auto_mode:=true`: kapal start di autonomous mode dan langsung mengikuti waypoint.

File yang menangani bagian utama:

- Joystick input: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_control/asv_control/simple_joystick_teleop.py`
- Mapping axis/button joystick: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_control/config/joystick.yaml`
- Switch manual/autonomous dari tombol joystick: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_control/asv_control/joystick_mode_manager.py`
- Pemilih command manual atau autonomous: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_control/asv_control/manual_autonomy_mux.py`
- Konversi `/cmd_vel` menjadi thrust propeller dan sudut servo: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_control/asv_control/cmd_vel_to_thrusters.py`
- Gerak kapal dari thrust propeller dan servo: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_control/asv_control/planar_pose_controller.py`
- Autonomous waypoint follower: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_navigation/asv_navigation/gps_waypoint_follower.py`
- Launch utama dan `auto_mode`: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_bringup/launch/full_system.launch.py`
- Launch lintasan A/B: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_bringup/launch/full_system_lintasan_a.launch.py` dan `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_bringup/launch/full_system_lintasan_b.launch.py`
- Kontrol visual ray LiDAR: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_description/scripts/configure_model_sdf.py`
- Collision kapal dan sensor contact: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_description/models/asv_kki_2026/model.sdf`
- Collision object world: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_gazebo/models/*/model.sdf`
- Bridge contact collision dan sensor: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_sensors/launch/sensors.launch.py`
- Ringkasan status collision: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_sensors/asv_sensors/collision_monitor.py`

Alur data kontrol:

```text
Manual:
/dev/input/js0 -> simple_joystick_teleop -> /cmd_vel_manual
  -> manual_autonomy_mux -> /cmd_vel
  -> cmd_vel_to_thrusters -> thrust propeller + servo steering
  -> planar_pose_controller -> pose kapal di Gazebo

Autonomous:
GPS/IMU/Odom/LiDAR -> gps_waypoint_follower -> /cmd_vel_auto
  -> manual_autonomy_mux -> /cmd_vel
  -> cmd_vel_to_thrusters -> thrust propeller + servo steering
  -> planar_pose_controller -> pose kapal di Gazebo
```

Waypoint autonomous tidak harus ditabrak tepat. Waypoint dianggap selesai jika kapal sudah cukup dekat atau sudah melewati garis waypoint dengan error kecil. Parameter utamanya ada di `gps_waypoint_follower.py`:

- `reach_radius_m`
- `segment_pass_margin_m`
- `gate_cross_track_radius_m`

Visual ray LiDAR dikontrol dengan `show_lidar_visual`, default-nya sekarang
`true` agar ray terlihat saat Gazebo dibuka. Ini hanya mengubah tampilan ray di
Gazebo, bukan mematikan sensor. Topic `/asv/lidar/scan` tetap aktif untuk
object detection.

## Build

```bash
cd /home/ammar/Documents/asv_simulation/asv_kki_2026_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Joystick/gamepad dibaca langsung dari Linux joystick device `/dev/input/js0` oleh node `simple_joystick_teleop`, jadi simulasi tidak wajib bergantung pada package `ros-jazzy-joy`.

## Jalankan Simulasi

Simulasi penuh lintasan A, default `auto_mode:=true` sehingga kapal langsung autonomous:

```bash
cd /home/ammar/Documents/asv_simulation/asv_kki_2026_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch asv_bringup full_system_lintasan_a.launch.py
```

Simulasi penuh lintasan B:

```bash
ros2 launch asv_bringup full_system_lintasan_b.launch.py
```

Start manual memakai joystick/gamepad:

```bash
ros2 launch asv_bringup full_system_lintasan_a.launch.py auto_mode:=false
ros2 launch asv_bringup full_system_lintasan_b.launch.py auto_mode:=false
```

Start autonomous tetapi joystick tetap aktif untuk switch mode/stop:

```bash
ros2 launch asv_bringup full_system_lintasan_a.launch.py auto_mode:=true use_joystick:=true
```

Launch terpisah:

```bash
ros2 launch asv_bringup sim.launch.py
ros2 launch asv_bringup sensors.launch.py
ros2 launch asv_bringup navigation.launch.py
ros2 launch asv_bringup joystick.launch.py
```

Jika memakai `sim.launch.py` langsung, default-nya lintasan A. Untuk lintasan B:

```bash
ros2 launch asv_bringup sim.launch.py \
  world:=/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_gazebo/worlds/kki_2026_lintasan_b.sdf \
  spawn_x:=-10.8 spawn_y:=-8.7 spawn_yaw:=1.5708
```

## Kontrol Joystick

Konfigurasi ada di:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_control/config/joystick.yaml`

Default mapping untuk stick Xbox di Linux:

- Left stick atas/bawah, Axis `1`: throttle maju/mundur.
- Left stick kiri/kanan, Axis `0`: steering kiri/kanan.
- Tombol `X`, Button `2`: opsional deadman jika `require_enable_button:=true`.
- Tombol `Y`, Button `3`: tahan untuk turbo.
- Tombol `RB`, Button `5`: switch ke autonomous mode.
- Tombol `LB`, Button `4`: kembali ke manual mode.
- Tombol `B`, Button `1`: stop / emergency stop.
- Tombol `A`, Button `0`: clear emergency stop.

Cara pakai manual mode dengan stick Xbox:

1. Jalankan simulasi manual dengan `auto_mode:=false`.
2. Gerakkan left stick atas/bawah untuk maju/mundur.
3. Gerakkan left stick kiri/kanan untuk belok.
4. Tahan tombol `Y` untuk mode turbo.
5. Tekan `RB` untuk pindah ke autonomous.
6. Tekan `LB` untuk balik ke manual.
7. Tekan `B` untuk emergency stop, lalu tekan `A` untuk clear stop.

Default simulasi sekarang memakai `require_enable_button: false`, jadi kapal
langsung merespons left stick. Kalau ingin mode lebih aman seperti deadman
switch, ubah `require_enable_button: true` di `src/asv_control/config/joystick.yaml`;
setelah itu tombol `X` harus ditahan agar command manual keluar.

Cek apakah tombol stick Xbox terbaca:

```bash
ros2 topic echo /joy
ros2 topic echo /asv/control/joystick_status
ros2 topic echo /cmd_vel_manual
ros2 topic echo /asv/control/mode_status
```

Kalau nomor axis/button berbeda di laptop tertentu, cek device Linux dengan:

```bash
jstest /dev/input/js0
```

Lalu sesuaikan angka `axis_linear_x`, `axis_angular_yaw`, `enable_button`, `auto_button`, dan tombol lain di `src/asv_control/config/joystick.yaml`.

Kalau arah maju/mundur atau kiri/kanan terbalik, balik tanda plus/minus pada
`scale_linear_x`, `scale_linear_turbo_x`, `scale_angular_yaw`, atau
`scale_angular_turbo_yaw` di file yang sama.

Alur kontrol:

`/dev/input/js0 -> simple_joystick_teleop -> /joy + /cmd_vel_manual -> manual_autonomy_mux -> /cmd_vel -> cmd_vel_to_thrusters -> GZ thrust + servo steering topics`

Pada mode auto, `/cmd_vel_auto` dari waypoint follower masuk ke mux yang sama. Node `cmd_vel_to_thrusters` menghitung:

- thrust kiri/kanan untuk propeller;
- sudut servo kiri/kanan untuk mengarahkan thrust;
- pivot differential thrust saat kapal hampir diam tetapi perlu berputar.

Startup mode dikontrol oleh argumen launch `auto_mode`:

- `auto_mode:=true`: `manual_autonomy_mux` mulai dari mode `auto`, lalu kapal mengikuti waypoint.
- `auto_mode:=false`: `manual_autonomy_mux` mulai dari mode `manual`, dan joystick nodes otomatis ikut dijalankan.

Ganti mode lewat terminal:

```bash
ros2 topic pub --once /asv/mode std_msgs/msg/String "{data: 'auto'}"
ros2 topic pub --once /asv/mode std_msgs/msg/String "{data: 'manual'}"
ros2 topic pub --once /asv/mode std_msgs/msg/String "{data: 'stop'}"
```

## Topic Utama

- `/asv/gps/fix`: `sensor_msgs/NavSatFix`
- `/asv/imu/data`: `sensor_msgs/Imu`
- `/asv/lidar/scan`: `sensor_msgs/LaserScan`
- `/asv/camera/front/image`: `sensor_msgs/Image`
- `/asv/camera/down/image`: `sensor_msgs/Image`
- `/asv/odom`: `nav_msgs/Odometry`
- `/cmd_vel_manual`: joystick manual command
- `/cmd_vel_auto`: waypoint follower command
- `/cmd_vel`: command yang masuk ke thruster converter
- `/model/asv_kki_2026/joint/left_propeller_joint/cmd_thrust`: thrust kiri
- `/model/asv_kki_2026/joint/right_propeller_joint/cmd_thrust`: thrust kanan
- `/asv/thrusters/left_steering/cmd_pos`: sudut servo thruster kiri, radian
- `/asv/thrusters/right_steering/cmd_pos`: sudut servo thruster kanan, radian
- `/asv/control/thruster_status`: status thrust dan servo
- `/asv/navigation/status`: status waypoint
- `/asv/sensors/status`: status sensor
- `/asv/collision/detected`: `std_msgs/Bool`, `true` saat contact sensor kapal menyentuh object
- `/asv/collision/status`: ringkasan collision aktif
- `/asv/collisions/hull`: kontak Gazebo mentah dari hull/deck kapal
- `/asv/collisions/left_thruster`, `/asv/collisions/right_thruster`: kontak thruster
- `/asv/collisions/left_propeller`, `/asv/collisions/right_propeller`: kontak propeller
- `/asv/collisions/lidar`, `/asv/collisions/front_camera`, `/asv/collisions/down_camera`: kontak komponen sensor
- `/kki/collisions/objects`: kontak mentah dari buoy, obstacle, target box, dock, dan marker start/finish
- `/asv/collision/kinematic_detected`: collision response dari `planar_pose_controller`
- `/asv/collision/kinematic_status`: nama object yang ditahan oleh collision response kinematik

## Waypoint

Waypoint default:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_navigation/config/kki_waypoints.yaml`

File itu mengikuti lintasan A. File khusus lintasan:

- `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_navigation/config/kki_waypoints_a.yaml`
- `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_navigation/config/kki_waypoints_b.yaml`

Ubah titik `x` dan `y` dalam meter relatif terhadap world ENU. Lintasan A start/docking di kanan bawah. Lintasan B start/docking di kiri bawah.

## Model Kapal

Model Gazebo aktif sekarang memakai hull VRX `roboboat02`, bukan lagi mesh `Final_TOT.3dm`, karena hull VRX lebih bersih dan lebih stabil untuk simulasi navigasi. Referensi utama:

- Hull kapal: `/home/ammar/Documents/asv_simulation/vrx/vrx_gz/models/roboboat02`
- CPU/dry box: `/home/ammar/Documents/asv_simulation/vrx/vrx_gz/models/roboboat_cpubox`
- Thruster, housing, mount, propeller: `/home/ammar/Documents/asv_simulation/vrx/vrx_gz/models/roboboat01`
- 3D LiDAR body dan tekstur: `/home/ammar/Documents/asv_simulation/vrx/vrx_urdf/vrx_gazebo/models/3d_lidar`

Model VRX diskalakan ke ukuran prototipe:

- Panjang kira-kira `0.831 m`, memenuhi minimum `>= 0.60 m`.
- Lebar kira-kira `0.574 m`, memenuhi minimum `>= 0.20 m`.
- Massa model: base `7.2 kg`, total dengan thruster/propeller sekitar `7.7 kg`, memenuhi minimum `>= 7 kg`.
- Propulsi: dua propeller/thruster mandiri dengan servo yaw kiri dan kanan.
- Sensor aktif: GPS, IMU/compass, LiDAR, kamera depan, dan kamera bawah.

Workspace ini tetap tidak menghapus atau mengubah file CAD asli:

`/home/ammar/Documents/asv_simulation/Final_TOT.3dm`

Housing/mount thruster sekarang berada pada `left_thruster_servo_link` dan `right_thruster_servo_link`. Link ini diputar oleh joint servo:

- `left_thruster_steering_joint`
- `right_thruster_steering_joint`

Mesh propeller ada di `left_propeller_link` dan `right_propeller_link`, parent-nya adalah link servo masing-masing. `cmd_vel_to_thrusters` mengubah `/cmd_vel` menjadi thrust propeller, sudut servo, dan kecepatan putar propeller visual. `planar_pose_controller` membaca thrust + servo tersebut untuk menghitung gerak kapal yang stabil di permukaan air. Posisi pivot servo berada di buritan pada `x=-0.150`, `y=+/-0.264`, `z=-0.090` m, dengan limit servo sekitar `+/-45 deg`.

Logika autonomous utama tetap waypoint berbasis GPS dan compass/IMU. Kalau origin GPS simulasi tidak cocok dengan waypoint lokal, node akan fallback ke `/asv/odom` agar simulasi tetap bisa jalan. LiDAR dipakai sebagai sensor tambahan untuk memperlambat atau melakukan stop-turn saat obstacle terlalu dekat.

## Collision dan Contact Detection

Collision geometry kapal dan object lintasan ada di:

- `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_description/models/asv_kki_2026/model.sdf`
- `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_gazebo/models/*/model.sdf`

Kapal aktif dibuat `static=true` dan digerakkan oleh `planar_pose_controller`.
Pilihan ini dipakai karena model kecil mudah flickering/tenggelam jika dibuat
rigid body bebas di permukaan wave mesh. Konsekuensinya, Gazebo tidak otomatis
mendorong balik kapal saat menabrak object. Untuk itu collision punya dua lapis:

1. Contact sensor Gazebo pada collision geometry kapal dan object lintasan.
2. Collision response kinematik di `planar_pose_controller.py` yang menghentikan
   dan sedikit memantulkan kapal saat hull masuk radius buoy, obstacle, target,
   atau dock.

Contact sensor kapal aktif pada:

- hull/deck;
- thruster kiri/kanan;
- propeller kiri/kanan;
- LiDAR;
- kamera depan dan bawah.

Object lintasan juga punya contact sensor pada topic `/kki/collisions/objects`:

- buoy merah/hijau/biru diameter `0.20 m`;
- obstacle kuning;
- target kotak hijau/biru `0.60 x 0.60 x 0.90 m`;
- dock finish.
- marker start/finish tipis.

Bridge contact Gazebo ke ROS 2 didefinisikan di:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_sensors/launch/sensors.launch.py`

Node ringkasan collision:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_sensors/asv_sensors/collision_monitor.py`

Cek collision:

```bash
ros2 topic echo /asv/collision/detected
ros2 topic echo /asv/collision/status
ros2 topic echo /asv/collision/kinematic_status
ros2 topic echo /kki/collisions/objects
```

Catatan: karena kapal digerakkan dengan pose controller, contact dipakai untuk
deteksi sentuhan/tabrakan dan controller menghentikan gerak kapal. Ini menjaga
simulasi stabil; bukan simulasi tumbukan dinamis penuh seperti rigid body bebas.

## Visualisasi LiDAR Gazebo

LiDAR sensor tetap berjalan selama `/asv/lidar/scan` aktif. Visual ray di Gazebo dikontrol terpisah lewat argumen launch `show_lidar_visual`.

Tampilkan ray LiDAR:

```bash
ros2 launch asv_bringup full_system_lintasan_a.launch.py show_lidar_visual:=true
```

Sembunyikan ray LiDAR tanpa mematikan sensor:

```bash
ros2 launch asv_bringup full_system_lintasan_a.launch.py show_lidar_visual:=false
```

Yang berubah hanya tag `<visualize>` pada `lidar_sensor` di temporary SDF yang dibuat saat launch. Topic `/asv/lidar/scan` tetap dipublish dan navigation tetap bisa memakai LiDAR.

Script konversi `Final_TOT.3dm` tetap disimpan sebagai cadangan kalau nanti ingin kembali memakai CAD sendiri:

```bash
cd /home/ammar/Documents/asv_simulation/asv_kki_2026_ws
python3 src/asv_description/scripts/convert_final_tot_3dm.py --activate
```

Untuk kualitas visual lebih tinggi, naikkan batas face, tetapi Gazebo bisa menjadi berat:

```bash
python3 src/asv_description/scripts/convert_final_tot_3dm.py --activate --max-faces 120000
```

Jika script melaporkan `rhino3dm is not installed`:

```bash
python3 -m pip install --user rhino3dm
python3 src/asv_description/scripts/convert_final_tot_3dm.py --activate
```

Jika file `.3dm` lain berisi NURBS/Brep tanpa render mesh, tessellate/export dulu dari Rhino, Blender, atau FreeCAD menjadi `.obj`, `.dae`, atau `.stl`, lalu letakkan hasilnya di:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_description/models/asv_kki_2026/meshes/`

## Wave Simulation

World memakai ocean mesh/material dari SINGABOAT-VRX:

`/home/ammar/Documents/asv_simulation/SINGABOAT-VRX/vrx/wave_gazebo/world_models/ocean_waves`

File referensi world-nya ada di:

`/home/ammar/Documents/asv_simulation/SINGABOAT-VRX/singaboat_vrx/worlds/open_ocean.world`

Parameter ombak aktif dibuat lebih tenang untuk area lomba 30 x 30 m:

- `wind_speed: 2.0`
- `amplitude: 0.025`
- `period: 3.5`
- `scale: 1.0`
- `steepness: 0.55`

Apakah air memengaruhi gerak kapal? Pada setup aktif ini, wave mesh memengaruhi
visual pose kapal melalui `planar_pose_controller`: kapal diberi heave, roll,
dan pitch kecil mengikuti parameter `wave_amplitude_m`, `wave_roll_deg`, dan
`wave_pitch_deg`. Air belum menjadi gaya buoyancy/hydrodynamics penuh dari
solver Gazebo. Ini sengaja dibuat stabil agar kapal tidak flickering/tenggelam
saat simulasi lintasan, joystick, dan autonomous dijalankan.

Karena plugin `libWavefieldModelPlugin.so` pada SINGABOAT adalah plugin Gazebo Classic, world GZ Sim/Harmonic ini memakai visual ocean SINGABOAT plus overlay crest gelombang lokal agar ombak terlihat. Untuk dynamic wave simulation GZ Sim, build plugin `gz-waves1-*` dari folder referensi:

```bash
cd /home/ammar/Documents/asv_simulation/asv_wave_sim/gz-waves
mkdir -p build
cd build
GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
make -j$(nproc)
```

Jika CMake berhenti di `CGALConfig.cmake`, install dependency CGAL dev terlebih dahulu. Setelah plugin tersedia, dynamic model berikut dapat dipakai di world:

`file:///home/ammar/Documents/asv_simulation/asv_wave_sim/gz-waves-models/world_models/ocean_waves`

## Menambah Sensor atau World

Sensor kapal utama didefinisikan di:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_description/models/asv_kki_2026/model.sdf`

Bridge topic sensor ada di:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_sensors/config/bridge_topics.yaml`

World lomba ada di:

- `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_gazebo/worlds/kki_2026_lintasan_a.sdf`
- `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_gazebo/worlds/kki_2026_lintasan_b.sdf`
- `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_gazebo/worlds/kki_2026_asv_course.sdf` adalah alias lintasan A.

Generator lintasan ada di:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_gazebo/scripts/generate_kki_courses.py`

Setelah menambah sensor Gazebo, tambahkan juga mapping bridge di launch:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_sensors/launch/sensors.launch.py`
