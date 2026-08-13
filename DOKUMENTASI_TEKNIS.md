# Dokumentasi Teknis ASV KKI 2026

Dokumen ini memuat bagian teknis yang sengaja tidak ditempatkan di README. Sistem aktif menggunakan ROS 2 Jazzy, Gazebo Sim 8, dan Nav2.

## Persyaratan Lingkungan

Workspace dikembangkan pada jalur berikut:

```text
/home/ammar/Documents/asv_simulation/asv_kki_2026_ws
```

`src/asv_bringup/launch/sim.launch.py` masih memakai root absolut `/home/ammar/Documents/asv_simulation` untuk menemukan resource VRX dan plugin gelombang. Direktori referensi berikut perlu tersedia pada mesin yang menjalankan konfigurasi saat ini:

```text
/home/ammar/Documents/asv_simulation/vrx
/home/ammar/Documents/asv_simulation/SINGABOAT-VRX
/home/ammar/Documents/asv_simulation/asv_wave_sim
```

Komponen utama yang diperlukan:

- ROS 2 Jazzy dan `colcon`;
- Gazebo Sim 8 atau Gazebo Harmonic dari instalasi ROS 2;
- Nav2, `robot_localization`, `ros_gz`, dan RViz 2;
- Python 3 beserta dependensi paket yang tercantum dalam `package.xml`.

## Build Workspace

```bash
cd /home/ammar/Documents/asv_simulation/asv_kki_2026_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Direktori `build/`, `install/`, dan `log/` adalah keluaran `colcon` dan sudah diabaikan oleh Git. Jangan mengedit isi direktori tersebut; lakukan perubahan pada `src/`, kemudian build ulang.

## Menjalankan Simulasi

Lintasan A dengan Gazebo GUI:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py
```

Lintasan B:

```bash
ros2 launch asv_bringup full_system_nav2_lintasan_b.launch.py
```

Mode tanpa GUI untuk komputer dengan sumber daya terbatas:

```bash
ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py headless:=true
```

RViz dapat dibuka dari terminal lain:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rviz2 -d src/asv_nav2/config/asv.rviz
```

Untuk menjalankan world dan model kapal tanpa misi Nav2:

```bash
ros2 launch asv_bringup sim.launch.py auto_mode:=false
```

Joystick dijalankan dari terminal kedua ketika mode manual dibutuhkan:

```bash
ros2 launch asv_bringup joystick.launch.py
```

Hentikan launch dengan `Ctrl+C`. Hindari perintah `kill -9` massal karena dapat menghentikan proses ROS 2 lain yang tidak berkaitan.

## Argumen Launch Penting

| Argumen | Default | Fungsi |
|---|---:|---|
| `headless` | `false` | Menjalankan server Gazebo tanpa GUI. |
| `auto_mode` | `true` | Memulai sumber kendali otonom. |
| `show_lidar_visual` | `false` | Menampilkan ray LiDAR di GUI pada full-system launch. |
| `use_navsat` | `false` | Mengaktifkan jalur lokalisasi global GPS/NavSat. |
| `enable_lidar_pointcloud` | `false` | Mengaktifkan bridge PointCloud2 untuk validasi VoxelLayer. |
| `nav2_start_delay_s` | `35.0` | Menunda Nav2 sampai simulator dan sensor siap. |
| `mission_start_delay_s` | `50.0` | Menunda mission supervisor sampai Nav2 aktif. |

## Struktur Paket

| Paket | Tanggung jawab utama |
|---|---|
| `asv_description` | Model kapal, URDF/Xacro, SDF, mesh, sensor, dan thruster. |
| `asv_gazebo` | World Lintasan A/B, pelampung, target, docking, laut, dan plugin gelombang. |
| `asv_sensors` | Bridge Gazebo-ROS untuk kamera, GPS, IMU, LiDAR, odometri, dan kontak. |
| `asv_control` | Mode manual/otonom, mux perintah, konversi ke thrust, dan gerak planar kapal. |
| `asv_perception` | Deteksi warna HSV, pemetaan survei, landmark semantik, dan jalur pilihan. |
| `asv_nav2` | Konfigurasi Nav2, planner, controller, costmap, collision monitor, docking, dan behavior tree. |
| `asv_navigation` | Waypoint lintasan, supervisor misi, logika misi, dan pencatatan metrik. |
| `asv_bringup` | Launch yang menyatukan simulator, sensor, persepsi, kontrol, dan navigasi. |

## Alur Runtime

```text
Gazebo world dan model kapal
  -> bridge sensor ROS 2
  -> lokalisasi dan pemetaan
  -> planner/controller Nav2
  -> mission supervisor
  -> /cmd_vel_auto
  -> mux kendali
  -> perintah thrust dan sudut kemudi
  -> planar pose controller
  -> pose kapal kembali diperbarui di Gazebo
```

Nav2 mulai setelah delay default 35 detik. Mission supervisor mulai setelah 50 detik, memuat waypoint sesuai lintasan, menjalankan navigasi, meminta foto pada area target, dan mengarahkan kapal ke docking.

## Misi dan Data Keluaran

Konfigurasi waypoint berada di:

```text
src/asv_navigation/config/kki_waypoints_a.yaml
src/asv_navigation/config/kki_waypoints_b.yaml
```

Keluaran runtime tidak dilacak oleh Git dan dapat dibuat ulang:

- `mission_captures/`: bukti gambar kamera depan dan kamera bawah;
- `mission_maps/`: occupancy map, landmark, dan speed mask;
- `mission_metrics/`: metrik perjalanan dalam format CSV.

README memakai salinan gambar terpilih di `docs/images/`, sehingga dokumentasi visual tetap tersedia tanpa bergantung pada folder keluaran runtime.

Script `reset_map.py` membuat ulang peta kosong Lintasan A dan menghapus daftar landmark lama. Jalankan hanya jika memang ingin memulai pemetaan baru karena hasil peta sebelumnya akan ditimpa.

```bash
python3 reset_map.py
```

Speed mask dapat dibuat ulang setelah pemetaan:

```bash
python3 src/asv_nav2/scripts/generate_speed_mask.py \
  --course a \
  --ws /home/ammar/Documents/asv_simulation/asv_kki_2026_ws
```

## Topik ROS 2 Utama

| Topik | Isi |
|---|---|
| `/asv/gps/fix` | Posisi GPS simulasi. |
| `/asv/imu/data` | Orientasi dan gerak IMU. |
| `/asv/lidar/scan` | LaserScan untuk obstacle dan costmap. |
| `/asv/camera/front/image` | Gambar kamera depan 1280 x 720. |
| `/asv/camera/down/image` | Gambar kamera bawah 640 x 480. |
| `/asv/odom` | Odometri dari Gazebo. |
| `/cmd_vel_auto` | Perintah kecepatan dari sistem otonom. |
| `/cmd_vel_manual` | Perintah kecepatan dari joystick. |
| `/asv/mission/status` | Ringkasan fase dan keberhasilan misi. |
| `/asv/collision/detected` | Status kontak kapal dengan objek. |

Daftar topik yang aktif dapat diperiksa dengan:

```bash
ros2 topic list
ros2 node list
```

## Pengujian

Unit test paket navigasi dan persepsi:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select asv_navigation asv_perception
colcon test-result --verbose
```

Smoke test visual pada 13 Agustus 2026 menggunakan `full_system_nav2_lintasan_a.launch.py` berhasil memuat Gazebo Sim 8, world Lintasan A, model ASV, bridge kamera/GPS/IMU/LiDAR, node Nav2, dan menghasilkan foto target permukaan baru. Pengujian ini membuktikan startup dan aliran kamera, tetapi bukan pernyataan bahwa seluruh misi dua lintasan selalu selesai tanpa tuning tambahan.

## Catatan Konfigurasi Saat Ini

- Lintasan A meneruskan `mission_maps/a/speed_mask.yaml` ke Nav2.
- Lintasan B belum meneruskan speed-mask khusus pada full-system launch.
- Argumen `use_joystick` pada full-system Lintasan A saat ini dideklarasikan tetapi belum memasukkan `joystick.launch.py`; jalankan joystick dari terminal terpisah.
- Beberapa path masih absolut, sehingga workspace belum sepenuhnya portabel ke direktori atau komputer lain.
