# Panduan Menjalankan Vision Tracker di STB Armbian

Panduan langkah demi langkah untuk mentransfer, menginstal dependensi, dan menjalankan modul visual tracking di STB Armbian Anda (`armbian.satriasangga.my.id`).

---

## 1. Perintah Transfer Folder via SCP (Jalankan di Terminal PC / Mac)

Buka terminal di komputer Anda, lalu jalankan perintah berikut untuk menyalin seluruh folder `stb_vision` ke home directory STB:

```bash
scp -r /Users/sangga/Documents/ai_mobil/stb_vision sangga@armbian.satriasangga.my.id:~/
```

> [!NOTE]
> Jika port SSH Anda menggunakan port khusus (bukan default 22), gunakan opsi `-P <nomor_port>`, contoh:
> `scp -P 2222 -r /Users/sangga/Documents/ai_mobil/stb_vision sangga@armbian.satriasangga.my.id:~/`

---

## 2. Masuk ke STB via SSH

Setelah proses copy selesai, login ke STB seperti biasa:

```bash
ssh sangga@armbian.satriasangga.my.id
```

---

## 3. Instalasi Dependensi di STB Armbian (Cukup Sekali)

Di prosesor ARM STB (Cortex-A53), cara tercepat dan paling stabil adalah menggunakan package manager APT bawaan Linux (hanya butuh waktu ~30 detik):

```bash
sudo apt update
sudo apt install -y python3-opencv python3-serial
```

*(Opsi alternatif jika ingin menggunakan pip / venv: `pip3 install -r ~/stb_vision/requirements.txt`)*

---

## 4. Persiapan Hardware & Izin Port Serial

1. **Colokkan Webcam USB & Kabel ESP32 ke 2 port USB di STB.**
2. **Cek apakah perangkat sudah terdeteksi di Linux:**
   ```bash
   # Cek kamera USB (biasanya muncul /dev/video0)
   ls -l /dev/video*

   # Cek serial ESP32 (biasanya muncul /dev/ttyUSB0 atau /dev/ttyACM0)
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```
3. **Berikan izin akses port serial agar tidak Permission Denied:**
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   *(Jika baru pertama kali ditambahkan ke grup dialout, lakukan re-login SSH atau jalankan `newgrp dialout`)*.

---

## 5. Menjalankan Vision Tracker di STB

Masuk ke folder proyek di STB:
```bash
cd ~/stb_vision
```

### Opsi A: Mode Default (KCF Tracker + Auto-Lock)
Sesuai blueprint, skrip akan memberi jeda 3 detik untuk Anda berdiri di depan webcam, lalu mengunci target di kotak tengah:
```bash
python3 vision_tracker.py
```

### Opsi B: Mode Deteksi Wajah Otomatis (Face Cascade)
Jika ingin kamera langsung mendeteksi wajah secara otomatis tanpa perlu countdown di awal:
```bash
python3 vision_tracker.py --mode face
```

### Opsi C: Mode Simulasi / Tes Kamera Saja (Dry-Run)
Jika ESP32 belum dicolok dan hanya ingin memastikan kamera membaca frame dan FPS di terminal:
```bash
python3 vision_tracker.py --dry-run
```

---

## 6. Contoh Tampilan Telemetri di Terminal SSH

Karena Anda mengakses STB lewat SSH tanpa monitor HDMI, program otomatis berjalan dalam **Mode Headless** dan menampilkan telemetri langsung di terminal Anda:

```text
=======================================================
      STB ARMBIAN VISION TRACKER (ROBOTICS ENGINE)     
 - Resolusi : 320x240 @ 30 FPS
 - Mode     : KCF
 - UI Mode  : HEADLESS (SSH Terminal)
=======================================================
[SERIAL] Sukses terhubung ke ESP32 pada: /dev/ttyUSB0
[SYSTEM] Pipeline kamera aktif. Memulai pelacakan...

[LOCKED_TRACKING] FPS: 29.4 | ErrX:  +18 px | ErrY:   -5 px | Serial: TX_OK
[LOCKED_TRACKING] FPS: 30.1 | ErrX:   +3 px | ErrY:   -2 px | Serial: TX_OK
[  TARGET_LOST  ] FPS: 30.0 | Mencari Target...         | Serial: TX_OK
```

- Tekan **`Ctrl + C`** di terminal untuk menghentikan program dengan aman (skrip otomatis mengirim sinyal `LOST\n` ke ESP32 dan mematikan kamera).

---

## 7. Tips Menjalankan Program Tanpa Putus (Background / Screen)

Agar program tetap berjalan meskipun koneksi SSH laptop Anda terputus atau ditutup:

1. Pasang alat `screen`:
   ```bash
   sudo apt install -y screen
   ```
2. Buat sesi baru:
   ```bash
   screen -S vision
   cd ~/stb_vision && python3 vision_tracker.py
   ```
3. Tekan **`Ctrl + A` lalu `D`** untuk melepas sesi (*detach*). Program akan tetap berjalan di background STB!
4. Untuk membuka kembali sesi kapan saja:
   ```bash
   screen -r vision
   ```
