# Panduan Menjalankan Vision Tracker di STB Armbian

Panduan lengkap untuk menjalankan modul visual tracking di STB Armbian dan **mengakses Live Stream Visual HUD dari browser Mac/PC di jaringan WiFi lokal yang sama**.

---

## 1. Fitur Baru & Status Sistem

- **Smart Body & Clothes Tracking (MOSSE Ultra-Fast)**: Melacak seluruh badan dan corak pakaian pengguna dengan filter korelasi adaptif MOSSE (~100-200 FPS di ARM Cortex-A53) yang sangat ringan dan efisien.
- **Estimasi Jarak Monokular (Meter)**: Menghitung perkiraan jarak kamera ke pengguna dalam satuan meter (`1.75 m`) dan sentimeter secara real-time via triangulasi optik monokular, lengkap dengan status jarak (`IDEAL FOLLOW`, `TERLALU DEKAT`, `JAUH`).
- **Profil Ukuran Target**:
  - `🧍 Badan Penuh (Full Body)`: Kotak 120x200 px untuk melacak kepala hingga celana/kaki di jarak 1.5 - 3 meter.
  - `👕 Badan Atas & Baju (Torso)`: Kotak 115x155 px (default) untuk melacak kepala, pundak, dada, dan baju secara penuh di jarak 1 - 2 meter.
  - `😀 Wajah Saja (Face)`: Kotak 85x105 px untuk fokus wajah.
- **Smart Auto-Snap & Panduan Siluet**: Saat countdown 3 detik, sistem otomatis mendeteksi posisi tubuh pengguna dan menampilkan siluet panduan badan & baju agar penguncian target presisi.
- **Transmisi Serial Komprehensif ke ESP32**: Format data serial mengirim deviasi dan jarak: `"X:<err_x>,Y:<err_y>,D:<dist_cm>\n"` (misal `X:+10,Y:-3,D:175\n` -> error X +10 px, Y -3 px, Jarak 175 cm).
- **Auto-Mirror (Selfie Mode) & Flip Vertikal**: Kamera otomatis di-mirror horizontal secara default.
- **Live Web Streaming HUD (Mac Access)**: Tampilan visual stream kamera real-time dengan telemetri lengkap (`http://192.168.36.150:8080`).
- **Kontrol Jarak Jauh Interaktif**: Tombol ganti Profil Target, ganti Mode (MOSSE/KCF/Wajah), Mirror, Flip, dan Kunci Ulang langsung dari browser Mac.

---

## 2. Dependensi yang Sudah Terpasang di STB

Semua dependensi berikut sudah terpasang dan siap pakai:
- `python3-opencv` / `opencv-contrib-python-headless` (OpenCV 5.0.0 dengan modul tracker lengkap)
- `pyserial` (Serial UART 115200 ke ESP32)
- `flask` (Web streaming server & REST API)
- Model Haar Cascade wajah lokal (`haarcascade_frontalface_default.xml`)
- User sudah terdaftar di grup `dialout` & `video`

---

## 3. Cara Menjalankan Program di STB

Masuk ke folder proyek di STB:

```bash
cd ~/stb_vision
```

### Cara Cepat (Helper Script):
```bash
./run.sh
# atau dengan mode tertentu:
./run.sh --mode mosse
./run.sh --dry-run
```

### Opsi A: Mode Default (KCF Tracker + Auto-Lock) + Web HUD
Hitung mundur 3 detik saat kamera menyala, lalu otomatis mengunci orang di tengah:
```bash
python3 vision_tracker.py
```

### Opsi B: Mode MOSSE (Ultra Fast ~200 FPS)
Sangat direkomendasikan untuk tracking gerakan cepat di STB Armbian dengan efisiensi daya maksimal:
```bash
python3 vision_tracker.py --mode mosse
```

### Opsi C: Mode Deteksi Wajah Otomatis (Face Detection)
Mendeteksi wajah manusia secara otomatis tanpa perlu countdown di awal:
```bash
python3 vision_tracker.py --mode face
```

### Opsi D: Mode Simulasi / Dry-Run (Tanpa Colok ESP32)
Jika ingin menguji kamera dan melihat tampilan di Mac terlebih dahulu sebelum ESP32 dipasang:
```bash
python3 vision_tracker.py --dry-run
```

---

## 4. Cara Melihat Visual Tracker dari Mac Anda (Satu WiFi)

Karena STB berjalan tanpa monitor HDMI (*headless*), Anda cukup membuka browser (Safari, Chrome, atau Firefox) di Mac Anda:

Buka alamat berikut di browser Mac:
```text
http://192.168.36.150:8080
```

> [!TIP]
> **Fitur di Halaman Web Mac Anda:**
> 1. **Live Video Feed**: Melihat frame kamera langsung lengkap dengan kotak target, garis deviasi, dan crosshair tengah layar.
> 2. **Telemetry Real-Time**: Status target (`LOCKED_TRACKING`, `LOCKING_IN_X.XS`, `SEARCHING`), FPS, deviasi X dan Y.
> 3. **Tombol "🎯 Kunci Ulang"**: Memicu hitung mundur 3 detik untuk mengunci ulang target dari Mac Anda.
> 4. **Tombol Ganti Mode**: Beralih antar KCF, MOSSE, dan Deteksi Wajah secara langsung (tombol aktif otomatis disorot).
> 5. **Snapshot Foto Langsung**: Anda juga bisa mengambil snapshot satu frame kapan saja via:
>    `http://192.168.36.150:8080/snapshot`
> 6. **API Status JSON**:
>    `http://192.168.36.150:8080/api/status`

---

## 5. Output di Terminal SSH (Headless Telemetri)

Terminal SSH Anda tetap menampilkan status telemetri yang bersih dan tidak banjir teks:

```text
============================================================
      STB ARMBIAN VISION TRACKER (ROBOTICS ENGINE)     
 - Resolusi : 320x240 @ 30 FPS
 - Mode     : KCF
 - Kamera   : Index 1 (/dev/video1)
 - UI Mode  : HEADLESS (SSH Terminal)
 - Web HUD  : http://192.168.36.150:8080  <-- Buka di browser Mac!
============================================================
[WEB] Server visualisasi aktif di port 8080
[SERIAL] Sukses terhubung ke ESP32 pada: /dev/ttyUSB0
[SYSTEM] Pipeline kamera aktif. Memulai pelacakan...

[LOCKED_TRACKING] FPS: 29.5 | ErrX:  +15 px | ErrY:   -4 px | Serial: TX_OK
[LOCKED_TRACKING] FPS: 30.1 | ErrX:   +2 px | ErrY:   -1 px | Serial: TX_OK
```

- Tekan **`Ctrl + C`** di terminal untuk menghentikan program dengan aman kapan saja.

---

---

## 7. Panduan Bench Test ESP32 (Servo Pan-Tilt & Sensor Laser VL53L0X)

Firmware lengkap ESP32 tersedia di [`esp32_servo_tracking/esp32_servo_tracking.ino`](esp32_servo_tracking/esp32_servo_tracking.ino).

### A. Skema Pengkabelan Pinout ESP32
| Komponen | Pin Modul | Pin ESP32 | Keterangan |
| :--- | :--- | :--- | :--- |
| **Servo Pan (Horizontal)** | Sinyal (Oranye/Kuning) | **GPIO 18** | PWM Servo Pan |
| | VCC (Merah) | **VIN (5V)** | Sumber daya servo |
| | GND (Cokelat/Hitam) | **GND** | Ground bersama |
| **Servo Tilt (Vertikal)** | Sinyal (Oranye/Kuning) | **GPIO 19** | PWM Servo Tilt |
| | VCC (Merah) | **VIN (5V)** | Sumber daya servo |
| | GND (Cokelat/Hitam) | **GND** | Ground bersama |
| **Sensor Laser VL53L0X** | **VCC / VIN** | **3.3V** | Daya sensor ToF |
| | **GND** | **GND** | Ground bersama |
| | **SDA** | **GPIO 21** | I2C Data |
| | **SCL** | **GPIO 22** | I2C Clock |

### B. Persiapan Arduino IDE
1. Pasang dua library via **Tools -> Manage Libraries...**:
   - `ESP32Servo` oleh Kevin Harrington
   - `Adafruit_VL53L0X` oleh Adafruit
2. Buka file `esp32_servo_tracking/esp32_servo_tracking.ino`.
3. Pilih Board **ESP32 Dev Module**, colok kabel USB, lalu klik **Upload**.

### C. Pengujian di Meja Kerja (Serial Monitor 115200)
Setelah upload, buka **Serial Monitor** pada kecepatan **115200 bps**. Anda akan melihat telemetri real-time:
```text
[STATUS: TERKUNCI ] Pan: 92° Tilt: 90° | Jarak: 84 cm (LASER) | Aksi: 🟢 [ZONA TENANG - STOP] (70-90cm)
[STATUS: TERKUNCI ] Pan: 92° Tilt: 90° | Jarak: 125 cm (LASER) | Aksi: ⬆️  [MAJU MENGEJAR] (>100cm)
[STATUS: TERKUNCI ] Pan: 92° Tilt: 90° | Jarak: 25 cm (LASER) | Aksi: 🛑 [EMERGENCY CUTOFF] (<30cm!)
```

Ketika kabel USB ESP32 dicolokkan ke STB, Python di STB akan otomatis mengirim data deviasi $X, Y$ dan estimasi jarak secara real-time!

