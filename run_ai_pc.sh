#!/usr/bin/env bash
# run_ai_pc.sh - Launcher Pelacak Manusia Presisi Tinggi 360 AI (Pop!_OS / Ubuntu)
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

echo "================================================================="
echo "   MEMULAI XIAOMI / DJI STYLE FULL-BODY AI TRACKER (PC ENGINE)  "
echo "   Web HUD : http://localhost:8080 (Buka di Browser!)           "
echo "================================================================="

# 1. Cek & Install library pendukung jika belum ada
python3 -c "import ultralytics; import flask" 2>/dev/null || {
    echo "[!] Menginstal library 'ultralytics' dan 'flask'..."
    pip3 install ultralytics opencv-python pyserial flask
}

# 2. Buka izin akses USB Serial ESP32
sudo chmod 666 /dev/ttyUSB* 2>/dev/null || true
sudo chmod 666 /dev/ttyACM* 2>/dev/null || true

# 3. Mencegah crash Qt XCB di Linux Wayland / headless
export QT_QPA_PLATFORM=offscreen

# 4. Jalankan AI Tracker dengan Web HUD bawaan di Port 8080
python3 ai_human_follower_pc.py "$@"
