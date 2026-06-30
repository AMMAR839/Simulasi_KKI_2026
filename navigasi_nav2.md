# Navigasi ASV KKI 2026 — Nav2 Full Stack

File ini menjelaskan arsitektur navigasi berbasis **Nav2** (Navigation2) yang digunakan di workspace:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws`

Sistem ini menggantikan custom `gps_waypoint_follower.py` dengan ekosistem navigasi resmi ROS2 (Nav2) yang jauh lebih robust, dengan dukungan path planning global, obstacle avoidance berbasis costmap, behavior tree untuk recovery otomatis, dan docking server berbasis LiDAR.

---

## Daftar Isi

1. [Gambaran Arsitektur](#arsitektur)
2. [Alur Data dan Topik](#alur-data)
3. [Lokalisasi GPS — Dual EKF](#lokalisasi-gps)
4. [Nav2 BT Navigator](#bt-navigator)
5. [NavFn Global Planner](#global-planner)
6. [Regulated Pure Pursuit Controller](#controller)
7. [Costmap 2D — Deteksi Obstacle LiDAR](#costmap)
8. [Nav2 Collision Monitor](#collision-monitor)
9. [OpenNav Docking Server](#docking-server)
10. [Mission Supervisor Nav2](#mission-supervisor)
11. [Behavior Tree XML](#behavior-tree)
12. [File Konfigurasi](#file-konfigurasi)
13. [Cara Menjalankan](#cara-menjalankan)
14. [Monitoring dan Debug](#monitoring)
15. [Tuning Parameter](#tuning)
16. [Kesimpulan](#kesimpulan)

---

## Arsitektur

Sistem navigasi Nav2 ASV KKI 2026 memiliki arsitektur berlapis sebagai berikut:

```
┌─────────────────────────────────────────────────────────────────┐
│          mission_supervisor_nav2.py (Orchestrator)              │
│  • Muat waypoints dari YAML                                     │
│  • Kirim ke Nav2 via nav2_simple_commander Python API           │
│  • Trigger foto kamera di waypoint tertentu                     │
│  • Trigger docking via DockRobot action                         │
└──────────────────────┬──────────────────────────────────────────┘
                       │ followWaypoints() / DockRobot action
                       ▼
┌─────────────────────────────────────────────────────────────────┐
│              Nav2 BT Navigator (nav2_bt_navigator)              │
│  • Jalankan Behavior Tree: NavigateThroughPoses                 │
│  • Replanning otomatis jika path gagal                          │
│  • Recovery: spin → backup → clear costmap                     │
└───────────┬───────────────────────────┬─────────────────────────┘
            │                           │
            ▼                           ▼
┌───────────────────┐       ┌───────────────────────────┐
│  NavFn Planner    │       │  OpenNav Docking Server   │
│  (path global)    │       │  (docking ke buoy biru)   │
│  Dijkstra di      │       │  LiDAR-guided approach    │
│  costmap          │       │  staging → dock           │
└─────────┬─────────┘       └───────────────────────────┘
          │ path
          ▼
┌─────────────────────────────────────────────────────────────────┐
│      Regulated Pure Pursuit Controller (nav2_controller)        │
│  • Ikuti path dari planner dengan lookahead distance            │
│  • Skala kecepatan saat ada obstacle (cost-regulated)           │
│  • Toleransi GPS yang realistis (xy_goal_tolerance=0.65 m)     │
└─────────────────────────┬───────────────────────────────────────┘
                          │ cmd_vel (ke velocity smoother)
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│    Nav2 Velocity Smoother → Nav2 Collision Monitor              │
│  • Smooth akselerasi/deselerasi                                 │
│  • STOP polygon 0.8m: obstacle → berhenti total                 │
│  • SLOWDOWN polygon 1.8m: obstacle → kecepatan ×0.5            │
│  • LIMIT polygon 3.0m: obstacle → max 0.30 m/s                 │
└─────────────────────────┬───────────────────────────────────────┘
                          │ /cmd_vel (ke thruster converter)
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│         cmd_vel_to_thrusters.py + planar_pose_controller.py     │
│  • Konversi cmd_vel ke thrust propeller kiri/kanan              │
│  • Gerak fisik kapal di Gazebo                                  │
└─────────────────────────────────────────────────────────────────┘
            ↑ Sensor masuk ke semua layer di atas:
            /asv/gps/fix → EKF → map frame
            /asv/lidar/scan → costmap + collision monitor
            /asv/imu/data → EKF heading
```

---

## Alur Data

### Topik Utama

| Topik | Arah | Keterangan |
|-------|------|-----------|
| `/asv/gps/fix` | Input | GPS fix dari Gazebo (NavSatFix) |
| `/asv/imu/data` | Input | IMU/compass dari Gazebo |
| `/asv/odom` | Input | Odometry simulasi Gazebo |
| `/asv/lidar/scan` | Input | LiDAR scan 2D |
| `/asv/camera/front/image` | Input | Kamera depan untuk foto kotak hijau |
| `/asv/camera/down/image` | Input | Kamera bawah untuk foto kotak biru |
| `/odometry/gps` | Internal | GPS → ENU odometry (dari navsat_transform) |
| `/odometry/filtered` | Internal | Output EKF — pose terfusi |
| `map → odom → base_link` | TF | Tree TF untuk Nav2 |
| `/cmd_vel_nav` | Internal | Output controller ke velocity smoother |
| `/cmd_vel_smoothed` | Internal | Output smoother ke collision monitor |
| `/cmd_vel` | Output | Ke thruster converter |
| `/asv/mission/status` | Output | Status misi lengkap |

### TF Tree

```
map
└── odom          ← EKF map (GPS global, map→odom TF)
    └── base_link  ← EKF odom (lokal, odom→base_link TF)
        ├── lidar_link
        ├── gps_link
        ├── imu_link
        ├── front_camera_link
        └── down_camera_link
```

---

## Lokalisasi GPS — Dual EKF

Nav2 memerlukan tiga frame TF: `map`, `odom`, dan `base_link`. Untuk ASV dengan GPS di laut terbuka, digunakan **Dual EKF** dari package `robot_localization`:

### EKF Lokal (`ekf_filter_node`)
- **Tujuan:** Menghasilkan TF `odom → base_link` dengan frekuensi tinggi dan akurat jangka pendek.
- **Input:** `/asv/odom` (dari Gazebo) + `/asv/imu/data` (heading)
- **Output:** `/odometry/filtered` dan TF `odom → base_link`
- **Config:** `src/asv_nav2/config/ekf_odom.yaml`
- **Frekuensi:** 30 Hz

### EKF Global (`ekf_filter_node_map`)
- **Tujuan:** Menghasilkan TF `map → odom` menggunakan GPS untuk posisi global.
- **Input:** `/odometry/gps` (dari navsat_transform) + `/asv/imu/data`
- **Output:** TF `map → odom`
- **Config:** `src/asv_nav2/config/ekf_map.yaml`
- **Frekuensi:** 10 Hz (GPS lebih lambat)

### NavSat Transform (`navsat_transform_node`)
- **Tujuan:** Mengkonversi `/asv/gps/fix` (GPS fix global) ke `/odometry/gps` (ENU lokal).
- **Datum GPS:** latitude=1.470000, longitude=102.110000 (pusat lapangan KKI)
- **Config:** `src/asv_nav2/config/navsat_transform.yaml`

Hasilnya: koordinat waypoint `x, y` di file YAML langsung sesuai dengan frame `map` — tanpa perlu konversi manual.

---

## Nav2 BT Navigator

**Package:** `nav2_bt_navigator`
**Config:** `src/asv_nav2/config/nav2_params.yaml` → section `bt_navigator`
**BT XML:** `src/asv_nav2/behavior_trees/kki_mission_bt.xml`

BT Navigator adalah **otak** dari Nav2. Ia menerima goal (pose/waypoint) dari mission supervisor dan menjalankan Behavior Tree untuk mencapai goal tersebut.

### Cara Kerja BT

```
NavigateThroughPoses Goal diterima
│
└── RecoveryNode (max 6 retries)
    ├── PipelineSequence (jalur normal)
    │   ├── RateController (1 Hz replanning)
    │   │   └── ComputePathThroughPoses ← NavFn Planner
    │   └── FollowPath ← RPP Controller
    │
    └── Recovery (jika gagal)
        ├── ClearCostmap (hapus obstacle palsu)
        ├── Spin (putar 90° untuk re-orient)
        ├── Wait (tunggu 5 detik)
        └── BackUp (mundur 0.30 m)
```

Dengan BT ini, jika kapal tersangkut atau path gagal, sistem otomatis mencoba recovery tanpa intervensi manusia.

---

## NavFn Global Planner

**Package:** `nav2_navfn_planner`
**Algoritma:** Dijkstra (lebih smooth dari A* untuk open space)
**Config:** `nav2_params.yaml` → section `planner_server`

NavFn menghitung path global dari posisi kapal saat ini ke waypoint tujuan melalui **costmap global**. Di laut terbuka (tanpa peta statis), semua sel di costmap adalah free space kecuali obstacle yang terdeteksi LiDAR secara real-time.

**Parameter kunci:**
- `tolerance: 1.0` — toleransi jarak ke goal (disesuaikan dengan akurasi GPS)
- `allow_unknown: true` — free space by default (tidak perlu peta statis)
- `use_astar: false` — gunakan Dijkstra untuk path lebih halus

---

## Regulated Pure Pursuit Controller

**Package:** `nav2_regulated_pure_pursuit_controller`
**Config:** `nav2_params.yaml` → section `controller_server`

RPP adalah controller terbaik untuk kapal/kendaraan tidak-holonomik. Ia mengikuti path global menggunakan titik lookahead yang adaptif.

### Cara Kerja RPP

```
Path dari NavFn
     │
     ▼
Temukan titik lookahead di path (jarak: lookahead_dist)
     │
     ▼
Hitung arc (kurva) dari posisi robot ke titik lookahead
     │
     ▼
Hitung cmd_vel (linear + angular) untuk mengikuti arc tersebut
     │
     ▼ Regulasi kecepatan:
     ├── Kurangi speed saat ada obstacle di costmap (cost-regulated)
     ├── Kurangi speed saat radius kurva kecil (curvature-regulated)
     └── Kurangi speed saat mendekati tujuan (approach scaling)
```

**Parameter kunci untuk ASV:**

| Parameter | Nilai | Keterangan |
|-----------|-------|-----------|
| `desired_linear_vel` | 0.50 m/s | Kecepatan jelajah |
| `lookahead_dist` | 1.5 m | Titik lookahead |
| `min_lookahead_dist` | 0.8 m | Minimum saat pelan |
| `max_lookahead_dist` | 3.0 m | Maksimum saat cepat |
| `approach_velocity_scaling_dist` | 1.5 m | Mulai melambat 1.5m sebelum goal |
| `xy_goal_tolerance` | 0.65 m | Jarak dianggap sampai goal |
| `transform_tolerance` | 0.5 s | Toleransi delay TF (GPS) |
| `allow_reversing` | false | Kapal tidak mundur |
| `use_rotate_to_heading` | false | Skip rotasi in-place |

---

## Costmap 2D — Deteksi Obstacle LiDAR

**Package:** `nav2_costmap_2d`
**Config:** `nav2_params.yaml` → section `local_costmap` dan `global_costmap`

Costmap adalah representasi 2D lapangan dalam bentuk grid sel, di mana setiap sel memiliki nilai cost (0=free, 254=obstacle, 100-253=inflation zone).

### Local Costmap
- **Frame:** `odom` (rolling window ikut kapal)
- **Ukuran:** 12 m × 12 m
- **Resolusi:** 0.10 m/sel
- **Update:** 5 Hz
- **Layers:**
  - `obstacle_layer` — tandai sel obstacle dari LiDAR scan
  - `inflation_layer` — perluas obstacle 0.80 m (keselamatan kapal)

### Global Costmap
- **Frame:** `map` (statis, seluruh lapangan)
- **Ukuran:** 60 m × 60 m (origin di -30, -30)
- **Resolusi:** 0.20 m/sel
- **Update:** 2 Hz
- **Inflation radius:** 1.00 m (lebih besar untuk planning jarak jauh)

### Footprint Kapal
Footprint kapal di costmap (dalam frame `base_link`):
```
[[0.50, 0.28], [0.50, -0.28], [-0.50, -0.28], [-0.50, 0.28]]
```
Ini merepresentasikan hull ASV ±0.5 m depan/belakang × ±0.28 m kiri/kanan.

---

## Nav2 Collision Monitor

**Package:** `nav2_collision_monitor`
**Config:** `src/asv_nav2/config/collision_monitor_params.yaml`

Collision Monitor adalah **lapisan keselamatan** terpisah dari controller. Ia mengintersep perintah `/cmd_vel` dari controller dan memodifikasinya berdasarkan data LiDAR real-time.

### Tiga Zona Keselamatan

```
Tampak atas kapal (kapal menghadap kanan →):

  ←─────────────────── 3.0 m ──────────────────────→
  ┌──────────────────────────────────────────────────┐
  │                 LIMIT Zone (3.0m)                │ → max 0.30 m/s
  │   ┌──────────────────────────────────────┐       │
  │   │          SLOWDOWN Zone (1.8m)        │ → ×0.5│
  │   │   ┌──────────────────────────┐       │       │
  │   │   │     STOP Zone (0.8m)    │ → STOP │       │
  │   │   │      ┌─────────┐        │       │       │
  │   │   │      │  KAPAL  │        │       │       │
  │   │   │      └─────────┘        │       │       │
  │   │   └──────────────────────────┘       │       │
  │   └──────────────────────────────────────┘       │
  └──────────────────────────────────────────────────┘
```

| Zona | Jangkauan | Aksi | Keterangan |
|------|-----------|------|-----------|
| STOP | 0.8 m depan | `cmd_vel.linear.x = 0` | Berhenti total |
| SLOWDOWN | 1.8 m depan | Speed × 0.5 | Setengah kecepatan |
| LIMIT | 3.0 m depan | Max 0.30 m/s | Batasi kecepatan |

Collision Monitor membaca langsung dari `/asv/lidar/scan` tanpa melewati costmap, sehingga respons keselamatannya lebih cepat.

---

## OpenNav Docking Server

**Package:** `opennav_docking`
**Config:** `src/asv_nav2/config/docking_params.yaml`

OpenNav Docking Server menangani manuver docking otomatis ke **bola biru** di ujung lapangan.

### Alur Docking

```
Mission Supervisor kirim: DockRobot(dock_id="kki_buoy_dock_a")
                │
                ▼
Docking Server lookup pose dock dari database:
  kki_buoy_dock_a → map frame: x=12.35, y=-9.70
                │
                ▼
Hitung staging pose (1.8 m barat dari dock):
  staging: x=10.55, y=-9.70
                │
                ▼
Nav2 navigasikan kapal ke staging pose
                │
                ▼
Docking Server kontrol approach pelan ke dock pose
  (threshold: 0.40 m dari dock)
                │
                ▼
Sukses → result.success = True
```

### Konfigurasi Dock

```yaml
kki_buoy_dock_a:
  type: "kki_blue_buoys"
  frame: "map"
  pose: [12.35, -9.70, 0.0]  # Lintasan A

kki_buoy_dock_b:
  type: "kki_blue_buoys"
  frame: "map"
  pose: [12.35, -9.70, 0.0]  # Lintasan B
```

---

## Mission Supervisor Nav2

**File:** `src/asv_navigation/asv_navigation/mission_supervisor_nav2.py`

Mission Supervisor adalah **orkestrator** seluruh misi menggunakan `nav2_simple_commander` Python API.

### Fase Misi

```
INIT → Tunggu Nav2 aktif (lifecycle check)
  │
SURVEY
  ├── Muat waypoints dari kki_waypoints_a.yaml
  ├── Konversi ke PoseStamped dalam frame map
  ├── navigator.followWaypoints(poses)
  └── Monitor feedback.current_waypoint:
      ├── wp = "surface_photo_green_box"  → foto kamera depan
      ├── wp = "underwater_photo_blue_box" → foto kamera bawah
      └── Cek proximity sebagai fallback
  │
DOCKING
  └── DockRobot(dock_id="kki_buoy_dock_a")
  │
DONE → Publish status misi lengkap
```

---

## File Konfigurasi

### Package `asv_nav2`

| File | Keterangan |
|------|-----------|
| `config/ekf_odom.yaml` | EKF lokal — `odom → base_link` |
| `config/ekf_map.yaml` | EKF global (GPS) — `map → odom` |
| `config/navsat_transform.yaml` | GPS Fix → ENU odometry |
| `config/nav2_params.yaml` | BT, RPP controller, NavFn, costmap, waypoint follower |
| `config/collision_monitor_params.yaml` | 3 zona polygon LiDAR |
| `config/docking_params.yaml` | OpenNav docking ke buoy biru |
| `behavior_trees/kki_mission_bt.xml` | BT dengan replanning + recovery |
| `launch/nav2_stack.launch.py` | Launch semua node Nav2 |

---

## Cara Menjalankan

### Jalankan dengan Nav2 (Sistem Baru)

```bash
cd /home/ammar/Documents/asv_simulation/asv_kki_2026_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Lintasan A dengan GUI Gazebo
ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py

# Tanpa GUI Gazebo (lebih ringan)
ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py headless:=true
```

### Jalankan dengan Sistem Lama (Backup)

```bash
ros2 launch asv_bringup full_system_lintasan_a.launch.py
```

### Timeline Startup

```
0 detik   → Gazebo + robot spawn + sensor bridge
35 detik  → Nav2 stack mulai (EKF, planner, controller, collision monitor, docking)
50 detik  → Mission Supervisor mulai
~60 detik → Nav2 lifecycle active → misi mulai otomatis
```

---

## Monitoring dan Debug

### Cek Node Aktif

```bash
ros2 node list | grep -E "bt_navigator|controller|planner|collision|docking|ekf"
```

### Status Misi

```bash
ros2 topic echo /asv/mission/status
ros2 topic echo /waypoint_follower/feedback
```

### TF Tree (penting untuk debug lokalisasi)

```bash
ros2 run tf2_tools view_frames
# Buka frames.pdf — harus ada: map → odom → base_link → lidar_link
```

### Lokalisasi GPS

```bash
ros2 topic echo /odometry/filtered    # Output EKF (posisi di frame map)
ros2 topic echo /odometry/gps         # GPS → ENU setelah NavSat transform
ros2 topic echo /asv/gps/fix          # GPS raw
```

### Path dan Costmap

```bash
ros2 topic echo /plan --once                 # Path dari NavFn
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /collision_monitor_state
```

### Mode Manual dan Auto

```bash
# Ganti ke auto
ros2 topic pub --once /asv/mode std_msgs/msg/String "{data: 'auto'}"

# Ganti ke manual
ros2 topic pub --once /asv/mode std_msgs/msg/String "{data: 'manual'}"

# Status
ros2 topic echo /asv/control/mode_status
ros2 topic echo /asv/control/thruster_status
ros2 topic echo /asv/mission/status
```

---

## Tuning Parameter

### Kecepatan Jelajah

Edit `src/asv_nav2/config/nav2_params.yaml`:

```yaml
FollowPath:
  desired_linear_vel: 0.50   # Naikkan ke 0.60 untuk lebih cepat
```

### Akurasi Goal Waypoint

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.65    # Turunkan ke 0.40 untuk lebih presisi
```

### Zona Collision Monitor

Edit `src/asv_nav2/config/collision_monitor_params.yaml`:

```yaml
stop_polygon:
  points: "[[0.80, 0.38], ...]"  # Naikkan ke 1.00 untuk lebih aman

approach_polygon:
  slowdown_ratio: 0.5            # Turunkan untuk lebih lambat dekat obstacle
```

### Inflation Costmap

```yaml
inflation_layer:
  inflation_radius: 0.80   # Naikkan untuk lebih jauh dari obstacle
  cost_scaling_factor: 3.0 # Naikkan untuk lebih menghindari obstacle
```

---

## Perbandingan Sistem Lama vs Nav2

| Fitur | Sistem Lama | Nav2 |
|-------|-------------|------|
| Waypoint following | Custom PD controller | Regulated Pure Pursuit + BT |
| Path planning | Langsung ke titik | NavFn global path |
| Obstacle avoidance | Custom LiDAR threshold | Costmap 2D + inflation |
| Recovery dari stuck | Tidak ada | Otomatis: spin/backup/clear |
| Collision safety | Custom threshold | Polygon zones (3 level) |
| Docking | LiDAR proximity mode | OpenNav Docking Server |
| Lokalisasi | GPS + fallback odom | Dual EKF (odom + map frame) |

---

## Kesimpulan

Sistem Nav2 ASV KKI 2026 mengintegrasikan semua komponen navigasi profesional ROS2 untuk menyelesaikan empat misi KKI:

1. **Menyusuri lintasan 10 pasang buoy** — Nav2 BT navigator + NavFn planner + RPP controller mengikuti waypoints dengan replanning dan recovery otomatis.

2. **Foto kotak hijau permukaan** — Mission supervisor monitor `current_waypoint` dari feedback Nav2, trigger kamera depan di waypoint `surface_photo_green_box`.

3. **Foto kotak biru bawah permukaan** — Kamera bawah di waypoint `underwater_photo_blue_box`.

4. **Docking ke bola biru** — OpenNav Docking Server melakukan approach otomatis ke koordinat buoy dengan staging pose 1.8 m dari target.

Selama semua fase, **Nav2 Collision Monitor** menjaga keselamatan kapal dengan tiga zona polygon LiDAR yang independen dari logika navigasi utama — sehingga kapal tidak akan bertabrakan bahkan jika planning atau controller bermasalah.
