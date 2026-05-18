# Logika Navigasi ASV KKI 2026

File ini menjelaskan logika autonomous yang aktif di workspace:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws`

## Status Ombak

World sudah memakai model air/ombak dari referensi:

- `/home/ammar/Documents/asv_simulation/asv_wave_sim`
- `/home/ammar/Documents/asv_simulation/SINGABOAT-VRX`

Model aktifnya ada di:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_gazebo/models/kki_ocean_waves/model.sdf`

Parameter ombak dibuat ringan supaya kapal prototipe tidak terlihat terlalu liar:

- `tile_size`: 30 m
- `amplitude`: 0.025 m
- `period`: 3.5 s
- `wind_speed`: 2.0
- `steepness`: 0.55

Catatan penting: simulasi ini tidak memakai global buoyancy langsung dari Gazebo, karena sebelumnya membuat kapal kecil mudah terbalik dan keluar sendiri. Sebagai gantinya, kapal digerakkan oleh model planar marine controller di:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_control/asv_control/planar_pose_controller.py`

Controller ini memberi efek ombak ringan ke pose kapal:

- heave naik-turun kecil di sumbu Z;
- roll kecil kiri-kanan;
- pitch kecil depan-belakang.

Jadi ombak sekarang memengaruhi visual/pose kapal secara terkendali. Ini belum sama dengan simulasi hidrodinamika penuh yang menghitung gaya gelombang ke hull, tetapi cukup stabil untuk latihan navigasi KKI dan kapal tidak lagi terbalik.

## Propeller Sebagai Penggerak Utama

Gerak kapal sekarang tidak langsung diambil dari `/cmd_vel`. Alur aktualnya:

1. Joystick atau autonomous menghasilkan command:
   - manual: `/cmd_vel_manual`
   - autonomous: `/cmd_vel_auto`

2. `manual_autonomy_mux` memilih mode aktif dan mengeluarkan:
   - `/cmd_vel`

3. `cmd_vel_to_thrusters` mengubah `/cmd_vel` menjadi:
   - `/model/asv_kki_2026/joint/left_propeller_joint/cmd_thrust`
   - `/model/asv_kki_2026/joint/right_propeller_joint/cmd_thrust`
   - `/asv/thrusters/left_steering/cmd_pos`
   - `/asv/thrusters/right_steering/cmd_pos`

4. `planar_pose_controller` membaca thrust propeller dan sudut servo itu, lalu menghitung gerak kapal.

Artinya kapal hanya bergerak jika ada thrust propeller. Jika command thrust nol, kapal akan melambat dan berhenti. Servo thruster juga ikut dihitung, sehingga arah dorong tidak hanya lurus.

Model gaya yang dipakai:

- thrust kiri dan kanan dikonversi menjadi gaya maju/lateral;
- sudut servo mengubah arah gaya propeller;
- beda thrust kiri-kanan menghasilkan yaw moment;
- drag linear, drag lateral, dan yaw damping menahan gerak agar tidak liar.

## Pengaturan Kecepatan

Kecepatan kapal dikendalikan berlapis agar bisa dinaikkan tanpa membuat navigasi liar:

- `gps_waypoint_follower.py`
  - `max_speed_mps`: batas kecepatan autonomous utama. Nilai sekarang `0.62 m/s`.
  - `min_speed_mps`: kecepatan minimum saat masih boleh bergerak pelan. Nilai sekarang `0.12 m/s`.
  - `speed` pada file waypoint YAML: batas kecepatan per titik. Titik lintasan utama dibuat lebih cepat, sedangkan foto dan docking tetap pelan.
  - `turn_slow_heading_rad`, `cross_track_slow_distance_m`, dan `approach_slow_distance_m`: faktor yang otomatis menurunkan speed saat kapal salah arah, keluar garis tengah, atau mendekati target.

- `cmd_vel_to_thrusters.py`
  - `max_forward_thrust_n`: batas thrust maju propeller. Nilai sekarang `48 N`.
  - `max_speed_cmd_mps`: skala konversi dari command speed ke thrust. Nilai sekarang `0.9 m/s`.

- `planar_pose_controller.py`
  - `effective_thrust_scale`: seberapa besar thrust propeller menjadi gaya gerak di simulasi. Nilai sekarang `0.13`.
  - `max_linear_speed_mps`: batas kecepatan fisik model kapal. Nilai sekarang `0.90 m/s`.

Cara menaikkan speed yang aman:

1. Naikkan dulu `max_speed_mps` di navigation sedikit demi sedikit, misalnya `0.62` ke `0.68`.
2. Jika kapal masih lambat tetapi heading stabil, naikkan `speed` pada waypoint lintasan utama.
3. Jika command speed sudah besar tetapi kapal masih berat bergerak, baru naikkan `effective_thrust_scale` kecil-kecil, misalnya `0.13` ke `0.14`.
4. Jangan menaikkan `max_forward_thrust_n` dan `effective_thrust_scale` sekaligus, karena kapal akan mudah overshoot dan melewati gate.
5. Untuk foto dan docking, biarkan speed waypoint tetap lebih rendah supaya kamera dan kontak docking lebih akurat.

## Sensor Yang Digunakan

Sensor aktif:

- GPS: `/asv/gps/fix`
- IMU/compass: `/asv/imu/data`
- Odometry fallback: `/asv/odom`
- LiDAR: `/asv/lidar/scan`
- Kamera depan: `/asv/camera/front/image`
- Kamera bawah: `/asv/camera/down/image`

GPS dan compass adalah sumber utama posisi dan heading. Jika GPS simulasi belum sesuai origin, node boleh fallback ke odometry supaya simulasi tetap bisa diuji. LiDAR digunakan sebagai sensor tambahan untuk obstacle/environment detection.

## Logika Waypoint

Node autonomous utama:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_navigation/asv_navigation/gps_waypoint_follower.py`

Waypoint lintasan:

- Lintasan A: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_navigation/config/kki_waypoints_a.yaml`
- Lintasan B: `/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_navigation/config/kki_waypoints_b.yaml`

Waypoint ditempatkan di center line antar buoy merah-hijau. Kapal diarahkan ke titik lookahead di jalur, bukan hanya mengejar titik satu per satu. Ini membantu kapal tetap masuk di tengah gate.

Waypoint tidak harus ditabrak tepat di koordinatnya. Node sekarang menganggap waypoint selesai jika:

- jarak kapal ke waypoint sudah masuk `reach_radius_m`, sekarang `0.65 m`; atau
- kapal sudah melewati garis waypoint dengan `segment_pass_margin_m`, sekarang `0.35 m`, dan cross-track error masih di dalam `gate_cross_track_radius_m`, sekarang `0.75 m`.

Setelah kondisi itu terpenuhi, `current_index` langsung naik ke waypoint berikutnya. Ini mengurangi kasus kapal berputar-putar mengejar satu titik yang sudah dilewati.

Kontrol heading:

- hitung target heading dari posisi kapal ke titik lookahead;
- hitung heading error dari compass/IMU;
- pakai kontrol P-D untuk membuat yaw rate command;
- speed diturunkan saat heading error besar, cross-track error besar, atau mendekati waypoint.

LiDAR:

- area depan dicek dengan sudut sempit;
- area kiri/kanan dicek untuk memilih arah menghindar;
- jika obstacle terlalu dekat, kapal stop-turn;
- jika obstacle mulai dekat, kapal diperlambat dan diberi bias belok.

LiDAR dibuat lebih ringan dengan parameter:

- `lidar_process_every_n_scans`: hanya proses 1 dari setiap 3 scan.
- `lidar_sample_stride`: hanya baca 1 dari setiap 3 beam/ray.
- `lidar_detection_max_distance_m`: abaikan objek lebih jauh dari `2.5 m`.
- `front_scan_half_angle_deg`: hanya sektor depan sempit yang dipakai untuk stop/slow utama.

Dengan cara ini node tetap efektif mendeteksi obstacle dekat di depan kapal, tetapi tidak memproses seluruh point LiDAR setiap update.

## Misi KKI Yang Dicakup

### 1. Menyusuri 10 pasang buoy merah-hijau

World berisi 10 pasang buoy merah-hijau, total 20 buoy. Waypoint A dan B mengikuti garis tengah antar buoy. Status waypoint dipublikasikan di:

`/asv/navigation/status`

### 2. Foto kotak hijau permukaan air

Waypoint khusus:

- Lintasan A: `surface_photo_green_box` di sekitar `x=-10.20, y=-5.40`
- Lintasan B: `surface_photo_green_box` di sekitar `x=10.20, y=-5.40`

Node mission supervisor menyimpan foto dari kamera depan saat kapal masuk radius target.

Status:

`/asv/mission/status`

Folder hasil foto:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/mission_captures`

### 3. Foto kotak biru bawah permukaan

Waypoint khusus:

- Lintasan A: `underwater_photo_blue_box` di sekitar `x=-11.00, y=-7.70`
- Lintasan B: `underwater_photo_blue_box` di sekitar `x=11.00, y=-7.70`

Node mission supervisor menyimpan foto dari kamera bawah saat kapal masuk radius target.

### 4. Docking dan menyentuh bola biru

Waypoint akhir:

- Lintasan A: `dock_contact_blue_buoys` di sekitar `x=12.15, y=-9.70`
- Lintasan B: `dock_contact_blue_buoys` di sekitar `x=-12.15, y=-9.70`

Bola docking biru berdiameter 20 cm. Mission supervisor menghitung jumlah bola biru yang tersentuh berdasarkan jarak pusat kapal terhadap posisi bola docking. Radius contact dibuat untuk mewakili setengah lebar kapal + radius bola + margin simulasi.

Status docking:

`/asv/mission/status`

Contoh status:

```text
track_complete=True surface_photo=yes underwater_photo=yes docking_touched=1 mission_complete=True
```

## Mode Manual dan Auto

Ganti auto:

```bash
ros2 topic pub --once /asv/mode std_msgs/msg/String "{data: 'auto'}"
```

Ganti manual:

```bash
ros2 topic pub --once /asv/mode std_msgs/msg/String "{data: 'manual'}"
```

Lihat status:

```bash
ros2 topic echo /asv/control/mode_status
ros2 topic echo /asv/control/thruster_status
ros2 topic echo /asv/control/planar_pose_status
ros2 topic echo /asv/navigation/status
ros2 topic echo /asv/mission/status
```

## Kesimpulan

Sistem autonomous saat ini dirancang untuk menyelesaikan empat misi Guidebook KKI ASV 2026:

1. mengikuti lintasan buoy lewat waypoint center line;
2. mengambil foto kotak hijau dengan kamera depan;
3. mengambil foto kotak biru bawah permukaan dengan kamera bawah;
4. docking ke area finish dan menghitung kontak bola biru.

Untuk simulasi lomba, rantai kontrolnya sudah lengkap. Batasannya adalah efek ombak masih berupa efek pose terkendali, bukan gaya hidrodinamika penuh ke hull. Pilihan ini sengaja dipakai agar kapal prototipe tidak terbalik dan navigasi tetap bisa diuji sampai selesai.
