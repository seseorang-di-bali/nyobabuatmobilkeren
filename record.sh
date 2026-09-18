#!/bin/bash
# Shortcut untuk merekam stream dari STB selama X detik (default: 10 detik)
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DURATION="${1:-10}"
OUTPUT="${2:-stb_recording.mp4}"

echo "=========================================================="
echo " [RECORDER] Merekam stream STB selama $DURATION detik..."
echo "=========================================================="
"$DIR/.venv/bin/python" "$DIR/record_stream.py" "$DURATION" "$OUTPUT"
