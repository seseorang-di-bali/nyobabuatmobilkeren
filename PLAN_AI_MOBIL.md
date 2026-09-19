# 🚀 MASTER BLUEPRINT: 100% NON-CAMERA FOLLOWER ROBOT (ACOUSTIC-RF RADAR)
### Mobil Aki 12V Autonomous: Pelacakan Suara Akustik + STB Pusat Komando + Luckfox Pico AI Sentry

**Filosofi Desain:**  
Sistem kemudi dan pelacakan **100% BEBAS DARI KAMERA**! Robot tidak lagi menggunakan penglihatan visual untuk mengikuti Anda, melainkan menggunakan **Sistem Radar Akustik-Radio Nirkabel (Acoustic-RF Triangulation)**. Sistem ini **kebal dari masalah bayangan, tidak peduli baju apa pun yang Anda pakai, tidak terpengaruh cahaya, dan mustahil salah mengenali lemari/furnitur**.

Sementara itu, **STB Armbian** dan **Luckfox Pico Mini** tetap diberdayakan penuh untuk fitur-fitur pintar tingkat tinggi (Pusat Kendali Web, Suara Robot Pintar, dan AI Obstacle Sentinel).

---

## 1. 🏗️ PEMBAGIAN TUGAS 4 PILAR SISTEM (WHO DOES WHAT?)

```
+---------------------------------------------------------------------------------------+
|                                1. TARGET MAJIKAN                                      |
|    Membawa SMART BEACON TAG (ESP32-C3 SuperMini + Pemancar Suara 40kHz di Saku/Sabuk) |
+---------------------------------------------------------------------------------------+
                |                                                     |
       (Sinyal Radio ESP-NOW)                                (Pulsa Suara Ultrasonik 40kHz)
       (Kecepatan Cahaya, 0 ms)                              (Merambat di Udara, 343 m/s)
                |                                                     |
                v                                                     v
+------------------------------------+               +----------------------------------+
|      ESP32 PENGENDALI UTAMA        | <============ | 3x SENSOR HC-SR04 DI BEMPER      |
|  - Kirim Ping Radio                |               | (Kiri, Tengah, Kanan)            |
|  - Hitung Selisih Waktu Tiba (Δt)  |               | Mengukur sudut arah & jarak nyata|
|  - Kontrol Gas, Belok, Rem BTS7960 |               +----------------------------------+
+------------------------------------+
       ^                     ^
       | Serial              | Serial UART
       v                     v
+------------------------+  +-------------------------------+
|    STB ARMBIAN 2GB     |  |   LUCKFOX PICO MINI (NPU)     |
|   (Pusat Komando & HP) |  |   (AI Sentry & Anti-Rintangan)|
| - Web Radar HUD di HP  |  | - NPU 0.5 TOPS                |
| - Virtual Joystick HP  |  | - Deteksi Rintangan Lantai    |
| - Suara Robot Berbicara|  | - Mode Penjaga Parkir (Sentry)|
| - Monitor Aki 12V      |  +-------------------------------+
+------------------------+
```

---

## 2. 🎯 SISTEM KEMUDI UTAMA: RADAR AKUSTIK-RF (100% NON-KAMERA)

### A. Komponen di Tangan / Saku Anda (Smart Beacon Tag)
Bentuknya kecil seperti gantungan kunci / remote alarm mobil mini:
1. **ESP32-C3 SuperMini** (Ukuran $2 \times 1.7$ cm, ada port USB-C dan radio ESP-NOW).
2. **1x Transduser Pemancar Ultrasonik 40kHz** (Can 'T' 16mm).
3. **Baterai / Powerbank:** Baterai saku kecil atau kabel USB-C ke powerbank saku Anda.

### B. Komponen di Bemper Depan Mobil Aki
Dipasang **3 buah Sensor Ultrasonik HC-SR04** berjajar selebar sasis mobil aki:
* **Sensor Kiri (Pin TRIG & ECHO):** Menghadap menyerong ke kiri luar ($15^\circ$).
* **Sensor Tengah (Pin TRIG & ECHO):** Menghadap lurus ke depan.
* **Sensor Kanan (Pin TRIG & ECHO):** Menghadap menyerong ke kanan luar ($15^\circ$).
* Jarak bentangan antara sensor kiri dan kanan: **40 – 50 cm**.

### C. Matematika Cara Kerjanya (Kecepatan Suara):
1. ESP32 mobil aki memancarkan paket radio nirkabel ESP-NOW: *"PING!"* (waktu tempuh = 0 milidetik).
2. Tag di saku Anda menerima sinyal radio, lalu seketika menembakkan pulsa suara mikro 40kHz tak terdengar.
3. Suara merambat di udara dengan kecepatan $343\text{ m/s}$.
4. **Perhitungan Jarak:** Dihitung dari waktu tempuh gelombang suara ($D = t \times 343\text{ m/s}$).
   * Jarak $> 120$ cm: Mobil maju menyusul.
   * Jarak $75 - 95$ cm: Mobil berhenti santai (*sweet spot*).
   * Jarak $< 40$ cm: Rem darurat mutlak!
5. **Perhitungan Arah Belok (Kiri / Kanan):**
   * Karena jarak sensor kiri dan kanan terpisah 40 cm, jika Anda bergeser ke kiri, suara akan sampai ke sensor kiri **sepersekian mikrodetik lebih awal** daripada sensor kanan ($\Delta t$).
   * ESP32 menghitung selisih waktu $\Delta t$:
     * Jika Sensor Kiri duluan $\rightarrow$ Anda di sebelah **KIRI**! Mobil belok kiri.
     * Jika Sensor Kanan duluan $\rightarrow$ Anda di sebelah **KANAN**! Mobil belok kanan.
     * Jika Sensor Tengah paling cepat dan kiri-kanan seimbang $\rightarrow$ Anda lurus di **TENGAH**!
   * **Hasil:** Mobil aki mengikuti pergerakan Anda dengan responsif tanpa butuh kamera sama sekali!

---

## 3. 🖥️ PERAN STB ARMBIAN (PUSAT KOMANDO, WEB HUD & SUARA ROBOT)

Karena tugas pelacakan sudah diambil alih oleh radar suara, CPU STB menjadi **0% beban** dan RAM 2GB-nya bisa dimaksimalkan untuk fitur-fitur canggih:

1. **Web Radar Dashboard di HP (`http://IP-STB:8080` via Wi-Fi):**
   * Anda bisa membuka HP saat berjalan di depan mobil.
   * Tampilan layar HP menampilkan **grafik radar**: titik posisi Anda (kiri/tengah/kanan), jarak akurat (meter), kecepatan roda mobil, dan status aki 12V.
2. **Virtual Touchscreen Joystick di HP:**
   * Jika mobil sedang tidak mengikuti Anda, atau Anda ingin memarkirkan mobil aki ke sudut ruangan, Anda cukup menyentuh joystick virtual di layar HP untuk menyetirnya dari jarak jauh!
3. **Sistem Suara Robot Pintar (Text-To-Speech / Voice Synthesizer):**
   * STB dihubungkan ke speaker kecil / klakson mobil aki.
   * Robot bisa berbicara secara otomatis:
     * *"Sinyal majikan terkunci. Siap berangkat!"*
     * *"Awas rintangan! Berhenti darurat."*
     * *"Baterai aki tersisa 11.8 Volt, mohon diisi daya."*
4. **Data Logger (Blackbox Telemetri):**
   * Mencatat data jarak tempuh dan konsumsi arus motor ke database.

---

## 4. 🧠 PERAN LUCKFOX PICO MINI (AI SENTRY & ANTI-RINTANGAN JALAN)

Karena Luckfox Pico Mini memiliki chip **NPU 0.5 TOPS** yang dirancang khusus untuk Artificial Intelligence, kita manfaatkan untuk fitur keselamatan jalan dan keamanan:

1. **Fitur Utama: AI Road Hazard & Obstacle Sentinel (Anti-Tabrak Lantai Cerdas)**
   * Sensor ultrasonik di bemper atas fokus mendengar tag di pinggang Anda, sehingga dia tidak bisa melihat rintangan kecil di lantai (misalnya: sandal, batu, kardus, lubang jalan, atau kucing/hewan peliharaan).
   * Kamera mini pada Luckfox dipasang menghadap ke lantai/jalan depan mobil.
   * NPU Luckfox menjalankan model deteksi rintangan (*Obstacle Classifier*).
   * **Hasilnya:** Saat mobil aki sedang melaju mengikuti Anda, jika di lantai tiba-tiba ada kucing tidur atau barang jatuh, NPU Luckfox langsung mengirim sinyal darurat: `HAZARD DETECTED!` ke ESP32 untuk **otomatis mengerem sebelum menabrak**!
2. **Fitur Sekunder: Smart Sentry Mode (Pengaman Mobil Saat Parkir)**
   * Saat mobil aki Anda parkir dan Anda tinggal pergi:
   * Luckfox Pico masuk ke mode penjaga (*Sentry Mode*).
   * Jika ada orang tak dikenal mendekati atau menyentuh mobil aki, NPU mendeteksi orang tersebut dan memerintahkan STB membunyikan alarm suara anti-maling!

---

## 5. 🔌 TOPOLOGI INTERKONEKSI SISTEM

| Dari Perangkat | Ke Perangkat | Jalur Komunikasi | Data yang Lewat |
|:---|:---|:---|:---|
| **Tag di Saku** | **ESP32 Mobil** | Radio ESP-NOW (Nirkabel) | Paket handshake "PING" & "ACK" |
| **3x HC-SR04** | **ESP32 Mobil** | Digital Pin (GPIO) | Pulsa gelombang suara 40kHz (Arah & Jarak) |
| **ESP32 Mobil** | **Driver BTS7960** | PWM 25kHz | Arus daya motor kiri & kanan (Maju/Belok/Rem) |
| **ESP32 Mobil** | **STB Armbian** | USB Serial (`115200 bps`) | Telemetri jarak, sudut, voltase aki, & status |
| **Luckfox Pico** | **ESP32 Mobil** | Serial UART TX/RX | Interupsi pengaman darurat (`EMERGENCY_STOP`) |
| **STB Armbian** | **Smartphone Anda** | Wi-Fi HTTP (`Port 8080`) | Web Radar HUD, Kontrol Joystick, & Status |

---

## 6. 🛒 DAFTAR BELANJA KOMPONEN RESMI

| No | Komponen | Jumlah | Estimasi Harga | Fungsi Utama |
|:---|:---|:---:|:---:|:---|
| 1 | **ESP32-C3 SuperMini** | 1 pcs | Rp 28.000 – Rp 35.000 | Otak Smart Tag di saku (Pemancar radio ESP-NOW) |
| 2 | **Sensor Ultrasonik HC-SR04** | 3 pcs | Rp 12.000 × 3 = Rp 36.000 | Array 3 mata penerima gelombang suara di bemper mobil |
| 3 | **Transduser Ultrasonik 40kHz (T Can 16mm)** | 1 pcs | Rp 5.000 – Rp 10.000 | Corong pemancar suara ultrasonik pada tag saku |
| 4 | **Driver Motor BTS7960 43A** | 1 pcs | Rp 45.000 – Rp 60.000 | Penggerak daya 2 motor aki 12V (Wajib, arus besar) |
| 5 | **Step-Down LM2596** | 1 pcs | Rp 8.000 – Rp 12.000 | Penurun aki 12V menjadi 5.0V stabil untuk mikrokontroler |
| 6 | **Speaker Mini 3W / Mini USB Speaker** | 1 pcs | Rp 10.000 – Rp 20.000 | Pengeras suara robot bersuara (Dicolok ke STB) |

> 💰 **TOTAL BIAYA BELANJA: HANYA SEKITAR Rp 130.000 – Rp 165.000!**  
> *(Semua komponen lain: Sasis mobil aki, aki 12V, STB Armbian, ESP32 utama, dan Luckfox Pico Mini sudah Anda miliki).*
