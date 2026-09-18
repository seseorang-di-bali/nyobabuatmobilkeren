#!/usr/bin/env python3
"""
Script Perekam Stream STB Otomatis (Ringan & Cepat):
Merekam video feed langsung dari STB Vision Tracker ke file MP4 lokal.
Ukuran file sangat kecil (~500 KB - 1 MB untuk 10 detik).
"""
import cv2
import time
import sys
import os

def record(duration=10, output_file="stb_recording.mp4", url="http://192.168.110.150:8080/video_feed"):
    print(f"[RECORDER] Menghubungi stream di {url}...")
    cap = cv2.VideoCapture(url)
    
    if not cap.isOpened():
        print(f"[ERROR] Tidak dapat membuka stream di {url}. Pastikan STB sedang aktif!")
        return False

    ret, frame = cap.read()
    if not ret or frame is None:
        print("[ERROR] Gagal membaca frame pertama dari stream.")
        cap.release()
        return False

    h, w = frame.shape[:2]
    fps = 20.0
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_file, fourcc, fps, (w, h))

    print(f"[RECORDER] Memulai perekaman selama {duration} detik ke {output_file} ({w}x{h} @ {fps}fps)...")
    start_time = time.time()
    frames_recorded = 0

    try:
        while (time.time() - start_time) < duration:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.02)
                continue
            out.write(frame)
            frames_recorded += 1
            elapsed = time.time() - start_time
            print(f"\r[RECORDER] Merekam... {elapsed:.1f}/{duration}s ({frames_recorded} frame)", end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[RECORDER] Perekaman dihentikan manual oleh user.")
    finally:
        cap.release()
        out.release()

    size_kb = os.path.getsize(output_file) / 1024 if os.path.exists(output_file) else 0
    print(f"\n[SELESAI] Rekaman tersimpan di '{output_file}' ({size_kb:.1f} KB, {frames_recorded} frame).")
    return True

if __name__ == "__main__":
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    out = sys.argv[2] if len(sys.argv) > 2 else "stb_recording.mp4"
    record(dur, out)
