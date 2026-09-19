#!/usr/bin/env bash
# run_ai_pc.sh - Launcher Pelacak Manusia Presisi Tinggi 360 AI (Pop!_OS / Ubuntu)
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

echo "================================================================="
echo "   MEMULAI XIAOMI / DJI STYLE FULL-BODY AI TRACKER (PC ENGINE)  "
echo "================================================================="

# 1. Cek & Install library ultralytics jika belum ada
python3 -c "import ultralytics" 2>/dev/null || {
    echo "[!] Menginstal library 'ultralytics' (YOLOv8 Full-Body AI)..."
    pip3 install ultralytics opencv-python pyserial
}

# 2. Buka izin akses USB Serial ESP32
sudo chmod 666 /dev/ttyUSB* 2>/dev/null || true
sudo chmod 666 /dev/ttyACM* 2>/dev/null || true

# 3. Jalankan AI Tracker
python3 ai_human_follower_pc.py "$@"
