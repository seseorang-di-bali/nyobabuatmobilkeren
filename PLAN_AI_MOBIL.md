Bertindaklah sebagai Senior Embedded Systems, Computer Vision, & Robotics Engineer. Saya sedang membangun "Autonomous Vision-Guided Following Robot" skala utilitas menggunakan sasis dan motor gearbox mobil aki anak bertenaga 12V. 

Saya memerlukan blueprint teknis lengkap beserta kode implementasi produksi yang modular, tangguh (fail-safe), bebas placeholder/pseudocode, dan siap dieksekusi langsung pada dua subsistem: Sisi Linux (STB Armbian) dan Sisi Mikrokontroler (ESP32).

Sistem ini mengawinkan kendali refleks perangkat keras dengan Machine Learning (Hybrid Imitation Learning / Behavioral Cloning) agar robot tidak sekadar dikendalikan aturan kaku (hardcoded), melainkan mampu merekam gaya kemudi manusia dan menyempurnakan kehalusan manuvernya secara mandiri.

---

### 1. SPESIFIKASI ARSITEKTUR & PERANGKAT KERAS

#### A. Komputasi & Kontrol
* **High-Level Brain:** STB Android TV (ZTE B860H / HG680P, Quad-Core Cortex-A53, RAM 2GB, OS Armbian Linux / Ubuntu ARM).
* **Low-Level Reflex Controller:** ESP32 DevKit V1 30-Pin (FreeRTOS / Non-blocking loop).
* **Interkoneksi Sistem:** USB Serial via `/dev/ttyUSB0` pada baud rate `115200 bps` (Auto-reconnect & packet checksum/format parsing).

#### B. Penggerak & Daya
* **Penggerak Utama:** 2x DC Motor 12V High-Torque bawaan mobil aki anak (Konfigurasi Differential Drive: Roda Kiri & Kanan).
* **Motor Driver:** Dual H-Bridge BTS7960 43A High Current (VCC Logika & Enable pin `R_EN`/`L_EN` ke 5V/3.3V).
  * Motor Kiri: `RPWM` -> GPIO 25, `LPWM` -> GPIO 26.
  * Motor Kanan: `RPWM` -> GPIO 27, `LPWM` -> GPIO 14.
* **Sumber Daya Utama:** Aki Kering VRLA 12V (Common Ground ke seluruh ground logika).
* **Regulator Daya:** Step-Down LM2596 (12V ke 5.0V stabil) untuk menyuplai ESP32 via pin VIN dan daya servo.

#### C. Sensor & Aktuator Penglihatan
* **Kamera:** USB Webcam UVC standar (Resolusi streaming 320x240 @ 30 FPS).
* **Gimbal Vision:** 2-Axis Pan-Tilt Servo (2x TowerPro SG90 micro servo) dikendalikan ESP32:
  * Pan (Horizontal): GPIO 18 (Rentang $20^\circ - 160^\circ$, netral $90^\circ$).
  * Tilt (Vertikal): GPIO 19 (Rentang $50^\circ - 130^\circ$, netral $90^\circ$).
* **Sensor Jarak Presisi:** Sensor Laser Time-of-Flight (ToF) VL53L0X pada bus I2C (SDA GPIO 21, SCL GPIO 22).
* **Sensor Dinamika Sasis:** Sensor IMU/Gyro MPU6050 pada bus I2C paralel (SDA GPIO 21, SCL GPIO 22).

---

### 2. KEBUTUHAN IMPLEMENTASI & FITUR PERANGKAT LUNAK

#### BAGIAN I: SISI LINUX / STB ARMBIAN (Python 3)
1. **Pipeline Visual Super Ringan:**
   * Hindari arsitektur Deep Learning berat (YOLO/PyTorch dilarang karena keterbatasan CPU).
   * Gunakan OpenCV Tracker berbasis **KCF (Kernelized Correlation Filters)** pada resolusi 320x240.
   * Mekanisme Auto-Lock: Hitung mundur 3 detik saat start, kunci bounding box otomatis di tengah layar ($100 \times 140$ px).
   * Hitung deviasi piksel: `err_x = target_cx - 160` dan `err_y = target_cy - 120`.
   * Format Serial: Kirim `X:<err_x>,Y:<err_y>\n` setiap frame valid, atau `LOST\n` jika tracking terlepas.
2. **Sub-Modul Imitation Learning (Shadow Driving & Dataset Logging):**
   * Mode rekam telemetri ke `driving_dataset.csv`: merekam korelasi antara `[Sudut_Pan, Jarak_ToF, Gyro_Yaw]` terhadap `[PWM_Kiri, PWM_Kanan]` dari kemudi manual operator.
   * Skrip training kilat (`train_brain.py`): Melatih model regresi ringan (`RandomForestRegressor` atau `Ridge` via `scikit-learn`) yang selesai dieksekusi dalam $< 10$ detik di STB.
   * Skrip inferensi otonom (`autonomous_runner.py`): Menjalankan model hasil training untuk menghasilkan kontrol kecepatan motor yang adaptif dan mulus.

#### BAGIAN II: SISI MIKROKONTROLER ESP32 (Arduino C++ / PlatformIO)
1. **Serial Parser Non-Blocking:**
   * Membaca string `X:...` dan menggerakkan servo Pan-Tilt dengan kontrol proporsional halus (Deadzone 12 piksel anti-jitter).
   * Menangani timeout serial: Jika tidak ada data serial dalam 500 ms, otomatis aktifkan status `targetLost`.
2. **Logika Translasi & Rotasi Sasis:**
   * Jika sudut Pan melenceng dari zona mati ($85^\circ - 95^\circ$), putar sasis di tempat untuk meluruskan badan robot ke arah servo.
   * Menjaga jarak konstan target berdasarkan ToF VL53L0X:
     * Jarak $> 100$ cm: Maju mengejar proporsional.
     * Jarak $70 - 90$ cm: Zona tenang (Stop).
     * Jarak $35 - 60$ cm: Mundur perlahan menjauh.
3. **Heading Lock (MPU6050):**
   * Mengoreksi perbedaan gesekan gir motor kiri dan kanan saat maju lurus agar robot tidak melenceng miring ke dinding.
4. **Safety Cutoff Layer Mutlak (Hardware Protection):**
   * Jika ToF mendeteksi jarak $< 30$ cm ATAU status serial `LOST` / timeout: **POTONG PWM KE 0 SEKETIKA**.

---

### OUTPUT YANG HARUS ANDA BUAT:

1. **Diagram Topologi Kelistrikan & Pengkabelan:** Pemetaan pinout lengkap dari aki, LM2596, BTS7960, motor aki, ESP32, servo, hingga sensor I2C.
2. **Kode Python STB Armbian:**
   * Instruksi instalasi dependensi OS & Python.
   * `vision_tracker.py` (Tracking KCF visual + transmisi serial + fail-safe).
   * `shadow_logger.py` & `train_brain.py` (Modul perekam dataset kemudi manual & training kilat AI).
3. **Kode Firmware ESP32 (C++):**
   * Daftar library Arduino resmi yang wajib diinstal.
   * Kode `.ino` utuh berisi penggerak motor BTS7960, filter I2C untuk MPU6050 & VL53L0X, kendali servo Pan-Tilt, dan safety emergency cutoff.
4. **SOP Pengujian Mandiri (Dry-Run Bench Test):** Panduan bertahap melakukan uji coba awal tanpa risiko robot menabrak atau motor berputar liar.
