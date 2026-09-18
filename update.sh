#!/usr/bin/env bash
# update.sh - Tarik kode terbaru dari GitHub dan restart Vision Tracker otomatis
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

echo "=========================================================="
echo " [UPDATE] Menarik kode terbaru dari GitHub..."
echo "=========================================================="
git pull origin main

echo ""
echo " [RESTART] Me-restart STB Vision Tracker..."
systemctl --user restart stb_vision

echo " [STATUS] Memeriksa status server..."
sleep 2
systemctl --user status stb_vision --no-pager

echo ""
echo "=========================================================="
echo " SUKSES! Vision Tracker sudah diperbarui dan aktif."
echo " Web HUD URL: http://$(hostname -I | awk '{print $1}'):8080"
echo "=========================================================="
