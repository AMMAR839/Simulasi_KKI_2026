# Panduan Menjalankan Simulasi Pemetaan (Mapping) & Navigasi Otonom KKI 2026

Dokumen ini menjelaskan langkah-langkah detail dan perintah terminal untuk menjalankan pemetaan secara live menggunakan sensor LiDAR (**Fase 1**), memproses batas kecepatan, dan menjalankan navigasi otonom teroptimasi menggunakan peta hasil pemindaian tersebut (**Fase 2**).

---

## 🖥️ Mode Headless Gazebo + RViz Visualizer (Sangat Direkomendasikan)

Jika PC/Laptop Anda terasa berat saat menjalankan Gazebo GUI, Anda bisa mematikan tampilan Gazebo (**headless**) tetapi tetap membuka **RViz2** secara visual untuk memantau pergerakan kapal, peta costmap, dan sensor LiDAR.

### Cara Menjalankan untuk FASE 1 (Mapping Run):

1. **Reset Peta ke Kondisi Kosong (Aman):**  
   ```bash
   python3 reset_map.py
   ```

2. **Terminal 1: Jalankan Simulasi Mapping (Gazebo Headless)**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py headless:=true
   ```

3. **Terminal 2: Jalankan RViz Visualizer**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 run rviz2 rviz2 -d src/asv_nav2/config/asv.rviz
   ```
   *Anda akan melihat peta putih (ruang bebas) dan buoy hitam (rintangan) tergambar secara dinamis di RViz seiring pergerakan kapal!*

---

### Cara Menjalankan untuk FASE 2 (Optimized Navigation Run):

1. **Terminal 1: Jalankan Simulasi Navigasi (Gazebo Headless)**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py headless:=true
   ```

2. **Terminal 2: Jalankan RViz Visualizer**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 run rviz2 rviz2 -d src/asv_nav2/config/asv.rviz
   ```
   *Kapal akan langsung memuat peta dari Fase 1 dan menggunakan Speed Filter secara optimal.*

---

## 🟢 FASE 1: Pemetaan Lintasan Baru (Mapping Run - Standard GUI)

Gunakan fase ini jika Anda ingin melakukan pemetaan baru dari awal menggunakan GUI simulator bawaan Gazebo.

### Langkah-langkah:

1. **Reset Peta ke Kondisi Kosong (Aman):**  
   Buka terminal di direktori workspace (`asv_kki_2026_ws`), lalu jalankan perintah:
   ```bash
   python3 reset_map.py
   ```

2. **Inisialisasi Environment ROS 2:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

3. **Jalankan Simulasi Mapping:**
   ```bash
   ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py
   ```
   *(Catatan: Tambahkan `headless:=true` di ujung perintah jika ingin menjalankan simulasi tanpa membuka GUI Gazebo simulator agar lebih ringan).*

4. **Biarkan Kapal Menyelesaikan Lap 1:**
   * Kapal akan secara otonom melintasi seluruh gerbang rintangan (Gate 8 s.d. Gate 1).
   * Kapal akan mendekati dan mengambil foto kotak hijau (`surface_box`) dan kotak biru (`underwater_box`).
   * Terakhir, kapal akan merapat otonom ke stasiun dermaga (*docking*).
   * **Peta Otomatis Diselamatkan:** Begitu kapal mendeteksi kontak fisik dengan stasiun docking, sistem akan otomatis memanggil service `/asv/mapping/save` untuk menyimpan peta terbaru ke `mission_maps/a/lap1_map.pgm` dan `.yaml`.

---

## 🛠️ PROSES: Membuat Peta Batas Kecepatan (Speed Mask)

Setelah Fase 1 selesai dan peta `lap1_map.pgm` berhasil tersimpan di folder `mission_maps/a/`, Anda perlu memperbarui peta batas kecepatan (*speed mask*) berdasarkan data rintangan baru.

### Perintah Terminal:
Di terminal yang sama, jalankan script Python berikut:
```bash
python3 src/asv_nav2/scripts/generate_speed_mask.py --course a --ws /home/ammar/Documents/asv_simulation/asv_kki_2026_ws
```
Script ini akan menghasilkan file `speed_mask.pgm` dan `speed_mask.yaml` di folder `mission_maps/a/` yang berisi batas perlambatan dinamis (melambat otonom hanya di sekitar gerbang dan tikungan).

---

## 🔵 FASE 2: Navigasi Teroptimasi (Optimized Navigation Run)

Gunakan fase ini untuk menjalankan kapal dengan kecepatan maksimal dan optimal menggunakan peta yang sudah Anda buat di Fase 1. Kapal akan melaju kencang di jalur lurus panjang, namun secara otomatis melambat saat menikung atau mendekati rintangan.

### Langkah-langkah:

1. **Inisialisasi Environment ROS 2:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

2. **Jalankan Simulasi Navigasi Teroptimasi:**
   ```bash
   ros2 launch asv_bringup full_system_nav2_lintasan_a.launch.py
   ```

### Mengapa Fase 2 Lebih Cepat & Optimal?
* **Pemuatan Peta Dasar (Preload):** `survey_mapper` mendeteksi keberadaan `lap1_map.yaml` dan langsung memuatnya di detik ke-0 (tidak mulai dari peta kosong).
* **Speed Filter Aktif:** Nav2 memuat `speed_mask.pgm` hasil pemrosesan peta nyata Anda. Kecepatan kapal akan otomatis naik hingga batas target maksimal (**`0.75 m/s` atau lebih**) di area terbuka lurus, namun melambat otonom ke batas aman (**`0.32 - 0.41 m/s`**) saat melewati rintangan sempit atau bermanuver.
