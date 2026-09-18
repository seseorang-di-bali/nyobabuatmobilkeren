#!/usr/bin/env bash
# run_mac.sh - Menjalankan Vision Tracker langsung di macOS menggunakan webcam Mac & Web HUD

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ ! -d ".venv" ]; then
    echo "[SETUP] Virtual environment belum ditemukan. Membuat .venv..."
    python3 -m venv .venv
    ./.venv/bin/pip install --upgrade pip
    ./.venv/bin/pip install "opencv-contrib-python>=4.8" flask pyserial
fi

echo "============================================================"
echo "  MENJALANKAN VISION TRACKER DI MACOS"
echo "  Webcam : Kamera Mac (FaceTime HD / USB Webcam)"
echo "  Web HUD: http://localhost:8080"
echo "============================================================"

exec ./.venv/bin/python vision_tracker.py --camera 0 --dry-run --web-port 8080 "$@"
