#!/usr/bin/env bash
# run_pc.sh - Helper script untuk PC Linux
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

echo "=========================================================="
echo " Memulai Vision Tracker di PC (Linux Test Drive)"
echo " Web HUD: http://localhost:8080"
echo "=========================================================="

python3 vision_tracker.py --dry-run "$@"
