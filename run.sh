#!/usr/bin/env bash
# run.sh - Helper script untuk menjalankan Vision Tracker di STB Armbian
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

echo "=========================================================="
echo " Memulai STB Armbian Vision Tracker..."
echo " Web HUD URL: http://$(hostname -I | awk '{print $1}'):8080"
echo "=========================================================="

python3 vision_tracker.py "$@"
