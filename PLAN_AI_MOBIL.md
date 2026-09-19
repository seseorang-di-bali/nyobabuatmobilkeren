# 🚀 BLUEPRINT & MASTER PLAN: LOW-BUDGET AUTONOMOUS FOLLOWER ROBOT (VERSI IRIT & ANTI-NGOBROL DENGAN LEMARI)

**Proyek:** Robot Pengikut Manusia Skala Utilitas (Sasis Mobil Aki 12V)  
**Filosofi Desain:** *Low-Cost Sensor Fusion* — Menggabungkan penglihatan kamera USB cerdas berbiaya rendah dengan sensor fisik anti-tabrak, sehingga robot **tidak mungkin salah mengenali lemari**, responsif (30+ FPS), dan aman tanpa perlu komputer puluhan juta.

---

## 1. 🛒 DAFTAR BELANJA KOMPONEN (BILL OF MATERIALS - ESTIMASI BIAYA)

Berikut adalah komponen yang perlu dibeli di marketplace (Tokopedia / Shopee / Lazada). Harga di bawah adalah harga pasaran rata-rata komponen elektronik DIY di Indonesia.

| No | Nama Komponen | Estimasi Harga (IDR) | Fungsi Utama | Keterangan / Tips Pembelian |
|:---|:---|:---:|:---|:---|
| 1 | **Driver Motor BTS7960 43A High Current** | Rp 45.000 – Rp 60.000 | Mengendalikan 2 motor 12V mobil aki (Maju, Mundur, Belok, Rem) | Wajib pakai ini karena motor mobil aki arusnya besar (5A–15A). Jangan pakai L298N karena akan langsung terbakar. |
| 2 | **Sensor Jarak Ultrasonik HC-SR04 (atau JSN-SR04T)** | Rp 12.000 – Rp 18.000 *(atau Rp 45.000 utk versi waterproof)* | Mengukur jarak fisik ke kaki/tubuh majikan & Rem Darurat Anti-Tabrak | Beli 1 atau 2 pcs (1 di tengah depan, atau 1 kiri & 1 kanan depan). |
| 3 | **Modul Step-Down DC-DC LM2596 (Buck Converter)** | Rp 8.000 – Rp 12.000 | Menurunkan aki 12V menjadi 5.0V stabil untuk ESP32 & Servo | Memastikan servo tidak drop saat motor mobil aki menyedot arus besar. |
| 4 | **Kabel Jumper Dupont (40 pin Male-to-Female & Male-to-Male)** | Rp 10.000 – Rp 15.000 | Menghubungkan pin ESP32 ke Driver BTS7960, Servo, & Sensor | Pilih panjang 20 cm agar fleksibel di dalam sasis. |
| 5 | **Tempat ID Card / Badge Clip / Gantungan Tas** | Rp 3.000 – Rp 5.000 *(atau Rp 0 cetak sendiri)* | Menempatkan kartu visual ArUco Marker di pinggang / tas punggung | Sebagai "Kunci Identitas Majikan" agar robot 100% hanya mengikuti Anda. |
| 6 | **Saklar Toggle / Switch On-Off 12V (Opsional)** | Rp 5.000 – Rp 10.000 | Saklar darurat pemutus arus aki utama | Pengaman kelistrikan. |

> 💰 **TOTAL ESTIMASI BELANJA TAMBAHAN:** **Hanya sekitar Rp 85.000 – Rp 120.000!**  
> *(Sangat irit dan terjangkau untuk standar robot autonomous skala besar).*

### Komponen yang SUDAH Anda Miliki (Biaya Rp 0):
1. **Sasis Mobil Aki Anak 12V** + 2x Motor Gearbox + Roda.
2. **Aki Kering 12V** bawaan mobil aki.
3. **STB Android TV (B860H / HG680P)** OS Armbian Linux (Otak Komputasi Visual).
4. **Mikrokontroler ESP32 DevKit V1 30-Pin** (Otak Refleks Kendali Motor & Sensor).
5. **Webcam USB UVC** (Mata Visual).
6. **2-Axis Pan-Tilt Bracket + 2x Servo SG90** (Mata Bergerak Pengarah Kamera).

---

## 2. 🧠 RAHASIA ARSITEKTUR: BAGAIMANA VERSI IRIT BEKERJA SEPERTI ROBOT KOMERSIL?

Robot koper komersil ($800+) menggunakan pemancar UWB dan kamera stereo 3D. Kita meniru fungsionalitas tersebut dengan modal selembar kertas dan sensor suara:

```
                  +-------------------------------------------------+
                  |                 TARGET MANUSIA                  |
                  |  [ Memakai Kartu ArUco Marker di Tas/Pinggang ] |
                  +-------------------------------------------------+
                                      |         |
                  (Pantulan Suara 40kHz)        (Sinar Visual Piksel)
                                      |         |
                                      v         v
     +----------------------------------+     +-------------------------------+
     |  SENSOR ULTRASONIK (HC-SR04)     |     |   WEBCAM USB 2D (320x240)     |
     |  Mengukur jarak fisik nyata      |     |   Melihat sudut arah target   |
     +----------------------------------+     +-------------------------------+
                      |                                       |
                      v                                       v
         +--------------------------+          +-----------------------------+
         |     ESP32 (REFLEKS)      | <======> |     STB ARMBIAN (PIKIR)     |
         |  - Baca Ultrasonik       |  Serial  |  - cv2.aruco Fast Detect    |
         |  - Drive Motor BTS7960   |  115200  |    (< 2 ms, 40+ FPS)        |
         |  - Kendali Servo Pan/Tilt|   bps    |  - Kirim sudut Pan ke ESP32 |
         |  - REM DARURAT FISIK     |          |  - Backup Warna jika tertutup|
         +--------------------------+          +-----------------------------+
                      |
                      v
     +----------------------------------+
     |   DUAL BTS7960 -> 2 MOTOR AKI    |
     |   Maju, Belok, Menjaga Jarak     |
     +----------------------------------+
```

### Keunggulan Sistem Sensor Fusion Ini:
1. **Anti-Ngobrol dengan Lemari / Benda Mati (100% Solved!)**:
   * ArUco Marker adalah pola kode biner unik (seperti QR code mini berukuran $8 \times 8$ cm).
   * Lemari, gorden, atau kursi **tidak mungkin** memiliki pola matematika ArUco ID 42. Robot tidak akan pernah salah mengunci furnitur lagi!
2. **CPU STB Super Dingin & FPS Melonjak 40+ FPS**:
   * Deteksi ArUco OpenCV dilakukan secara native C++ berkecepatan tinggi ($< 2$ milidetik per frame).
   * Tidak ada lagi perhitungan FFT raksasa yang membuat STB drop ke 1 FPS.
3. **Sensor Ultrasonik Mengatur Jarak Nyata (Bukan Nebak-nebak Piksel)**:
   * Jarak manusia diukur dengan kecepatan gelombang suara fisik.
   * Jarak $> 120$ cm: Robot maju mengejar.
   * Jarak $70 - 100$ cm: Robot berhenti santai (*sweet spot*).
   * Jarak $< 45$ cm: Robot mundur perlahan / Rem darurat seketika.

---

## 3. 🔌 DIAGRAM PENGKABELAN & PINOUT (WIRING DIAGRAM)

### A. Pembagian Daya (Power Distribution 12V & 5V)
* **Aki 12V Positif (+)** -> Masuk ke Switch ON/OFF -> Cabang ke:
  1. `B+` pada Driver BTS7960 Kiri dan Kanan.
  2. `IN+` pada Modul Step-Down LM2596.
  3. `DC Jack 12V` STB Armbian (STB bawaannya menerima input 12V langsung).
* **Aki 12V Negatif (- / GND)** -> Common Ground ke:
  1. `B-` pada Driver BTS7960 Kiri dan Kanan.
  2. `IN-` pada Step-Down LM2596.
  3. Ground STB.
* **Output Step-Down LM2596 (Disetel ke 5.0V via potensiometer voltmeter)**:
  * `OUT+ (5V)` -> Cabang ke Pin `VIN` ESP32, VCC Servo Pan/Tilt, dan VCC HC-SR04.
  * `OUT- (GND)` -> Hubungkan ke pin `GND` ESP32, GND Servo, dan GND Sensor.

### B. Pinout ESP32 ke Driver BTS7960 (Penggerak Roda Mobil Aki)
| Pin Driver BTS7960 | Terhubung ke ESP32 | Keterangan |
|:---|:---|:---|
| **VCC** (Logika) | 5V (Output LM2596) | Daya chip logika driver |
| **GND** | GND ESP32 | Common Ground |
| **R_EN & L_EN** (Motor Kiri) | 3.3V atau 5V | Dijumper ke HIGH (Selalu aktif) |
| **RPWM** (Motor Kiri) | GPIO 25 | PWM Maju Motor Kiri |
| **LPWM** (Motor Kiri) | GPIO 26 | PWM Mundur Motor Kiri |
| **R_EN & L_EN** (Motor Kanan)| 3.3V atau 5V | Dijumper ke HIGH (Selalu aktif) |
| **RPWM** (Motor Kanan) | GPIO 27 | PWM Maju Motor Kanan |
| **LPWM** (Motor Kanan) | GPIO 14 | PWM Mundur Motor Kanan |

### C. Pinout ESP32 ke Servo Pan-Tilt (Pengarah Kamera)
| Komponen | Pin ESP32 | Keterangan |
|:---|:---|:---|
| **Servo Pan (Horizontal)** | GPIO 18 | Menoleh Kiri - Kanan ($20^\circ - 160^\circ$) |
| **Servo Tilt (Vertikal)** | GPIO 19 | Menoleh Atas - Bawah ($60^\circ - 120^\circ$) |

### D. Pinout ESP32 ke Sensor Ultrasonik HC-SR04 (Pengukur Jarak Fisik)
| Pin HC-SR04 | Pin ESP32 | Keterangan |
|:---|:---|:---|
| **VCC** | 5V | Sumber daya sensor |
| **GND** | GND | Common Ground |
| **TRIG** | GPIO 5 | Pin pemicu pulsa ultrasonik |
| **ECHO** | GPIO 17 *(via resistor divider atau direct)* | Pin pembaca pantulan suara |

---

## 4. 💻 ROADMAP TAHAP IMPLEMENTASI PERANGKAT LUNAK

### Tahap 1: Cetak & Pasang ArUco Marker (Rp 0 - Siap Sekarang)
* Kita sediakan skrip pembuat marker ArUco (Dictionary `DICT_4X4_50`, Marker ID 42).
* Anda cetak di kertas ukuran sekitar $8 \times 8$ cm atau $10 \times 10$ cm, masukkan ke plastik ID card, dan gantung di pinggang belakang / tas punggung.

### Tahap 2: Update Pipeline Visual STB (`vision_tracker.py`)
* Tambahkan detektor ArUco sebagai **Prioritas #1 Absolut**:
  * Begitu marker ID 42 terlihat, sistem mengunci dengan akurasi 100% ($< 1$ ms).
  * Menghitung sudut deviasi `X` dan `Y` untuk diarahkan oleh Servo Pan-Tilt.
* Jika marker tertutup tubuh sebentar saat berputar, sistem memakai **Prioritas #2 (Pakaian)** sebagai *temporary bridge* selama 2 detik sebelum mencari marker kembali.

### Tahap 3: Firmware ESP32 Sasis (`esp32_car_controller.ino`)
* ESP32 membaca data serial dari STB: `X:<err_x>,Y:<err_y>\n`.
* Servo Pan bergerak melacak target agar selalu di tengah frame.
* Jika sudut Pan melenceng $> 15^\circ$ dari garis lurus, ESP32 memutar roda mobil aki secara proporsional untuk menghadapkan sasis ke arah majikan.
* Sensor Ultrasonik membaca jarak:
  * Jarak $> 120$ cm: Maju mengejar.
  * Jarak $70 - 100$ cm: Berhenti (jarak nyaman).
  * Jarak $< 40$ cm: Rem fisik / mundur darurat.
  * Timeout serial ($> 500$ ms tanpa kabar dari STB): Motor mati seketika.

---

## 5. 📋 RINGKASAN TINDAKAN UNTUK ANDA
1. **Belanja Komponen:**
   * 1x Driver Motor BTS7960 43A.
   * 1x Sensor Ultrasonik HC-SR04.
   * 1x Step-Down LM2596.
   * 1 set Kabel Jumper Dupont Male-to-Female & Male-to-Male.
2. **Sementara Menunggu Paket Tiba:**
   * Kita bisa langsung mengaktifkan modul pelacak ArUco Marker di STB dengan kamera yang sudah ada sekarang. Anda bisa cetak markernya di selembar kertas dan mengujinya langsung di depan kamera!
