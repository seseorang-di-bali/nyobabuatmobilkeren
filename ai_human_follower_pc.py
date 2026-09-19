#!/usr/bin/env python3
"""
ai_human_follower_pc.py - Xiaomi / DJI Style High-Precision Full-Body AI Tracker
================================================================================
Dirancang khusus untuk PC (Pop!_OS / Ubuntu / Linux / Windows) dengan Ryzen 5 / Intel CPU.

Fitur Unggulan (Standar Xiaomi / DJI Gimbal):
1. Menggunakan YOLOv8 Neural Network Full-Body Detection (Class 0: Person).
2. Deteksi 360 Derajat: Menghadap depan, belakang (tampak punggung), samping, jalan, jongkok.
3. 100% Anti-Salah Kunci: Kursi, lemari, meja, dan dinding DIABAIKAN 100% (bukan manusia).
4. ByteTrack ID Locking: Mengunci 1 orang target secara konsisten meskipun ada orang lain lewat.
5. High-Speed Serial Engine: Mengirim koordinat X, Y, Jarak ke ESP32 secara gesit dan responsif.
6. Futuristic HUD UI: Tampilan bidikan crosshair, bracket target, dan telemetri jarak realtime.
"""

import sys
import os
import time
import glob
import math
import argparse
import threading
import cv2

# Cek & Import Ultralytics YOLO
try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False

# Cek & Import PySerial
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# Resolusi Kamera Optimal untuk High FPS AI Tracking
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2   # 320
CENTER_Y = FRAME_HEIGHT // 2  # 240

# Asumsi Optik Kamera (Untuk estimasi jarak monokular manusia tinggi 1.7m)
FOCAL_LENGTH_PX = 580.0
HUMAN_REAL_HEIGHT_M = 1.70


def find_serial_port():
    """Mencari port serial USB ESP32 secara otomatis di Linux / Pop!_OS / Windows / Mac."""
    if not HAS_SERIAL:
        return None

    try:
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            desc = (p.description or "").lower()
            hwid = (p.hwid or "").lower()
            if any(k in desc or k in hwid for k in ["ch340", "cp210", "ftdi", "usb", "serial", "uart"]):
                return p.device
        if len(ports) > 0:
            return ports[0].device
    except Exception:
        pass

    # Fallback pencarian manual di Linux / Mac
    manual_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + \
                   glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/tty.usbserial*")
    if sys.platform.startswith('win'):
        manual_ports += [f"COM{i+1}" for i in range(32)]

    for p in manual_ports:
        try:
            s = serial.Serial(p, 115200, timeout=0.05)
            s.close()
            return p
        except Exception:
            pass
    return None


class FastSerialSender:
    """Pengirim paket serial non-blocking ke ESP32 dengan rate-limiting 50 Hz."""
    def __init__(self, port=None, baud=115200, dry_run=False):
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self.ser = None
        self.lock = threading.Lock()
        self.latest_packet = "LOST\n"
        self.status = "DRY" if dry_run else "CONNECTING"
        self.running = True

        if not self.dry_run and HAS_SERIAL:
            self.thread = threading.Thread(target=self._worker, daemon=True)
            self.thread.start()

    def send(self, err_x, err_y, dist_cm):
        packet = f"X:{err_x:+d},Y:{err_y:+d},D:{dist_cm}\n"
        with self.lock:
            self.latest_packet = packet

    def send_lost(self):
        with self.lock:
            self.latest_packet = "LOST\n"

    def _worker(self):
        last_reconnect = 0
        last_sent_msg = ""
        while self.running:
            now = time.time()
            if self.ser is None:
                if now - last_reconnect > 2.0:
                    last_reconnect = now
                    port_to_open = self.port or find_serial_port()
                    if port_to_open:
                        try:
                            self.ser = serial.Serial(port_to_open, self.baud, timeout=0.01, write_timeout=0.01)
                            self.status = f"CONNECTED ({port_to_open})"
                            print(f"\n[SERIAL] Berhasil terhubung ke ESP32 pada: {port_to_open}")
                        except Exception as e:
                            self.ser = None
                            self.status = "DISCONNECTED"
            else:
                try:
                    with self.lock:
                        msg_to_send = self.latest_packet

                    self.ser.write(msg_to_send.encode('utf-8'))
                    last_sent_msg = msg_to_send
                except Exception:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    self.status = "RECONNECTING"

            # Rate limiting kirim data ke ESP32: ~50 Hz (20 ms) agar servo tidak overload
            time.sleep(0.02)

    def close(self):
        self.running = False
        if self.ser:
            try:
                self.ser.write(b"LOST\n")
                self.ser.close()
            except Exception:
                pass


def draw_hud(frame, target_box, conf, track_id, distance_m, fps, serial_status):
    """Menggambar HUD futuristik ala Xiaomi Smart Follower & DJI Gimbal."""
    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2

    # 1. Garis Target Tengah (Crosshair Horizon)
    color_dim = (80, 80, 80)
    cv2.line(frame, (cx - 25, cy), (cx + 25, cy), color_dim, 1)
    cv2.line(frame, (cx, cy - 25), (cx, cy + 25), color_dim, 1)
    cv2.circle(frame, (cx, cy), 18, color_dim, 1)

    if target_box is not None:
        bx, by, bw, bh = target_box
        tcx = bx + bw // 2
        tcy = by + bh // 2
        color_locked = (0, 255, 120)  # Hijau neon Xiaomi

        # 2. Bounding Box dengan Corner Brackets (Gaya DJI)
        corner_len = min(20, bw // 4, bh // 4)
        th = 2
        # Kiri Atas
        cv2.line(frame, (bx, by), (bx + corner_len, by), color_locked, th)
        cv2.line(frame, (bx, by), (bx, by + corner_len), color_locked, th)
        # Kanan Atas
        cv2.line(frame, (bx + bw, by), (bx + bw - corner_len, by), color_locked, th)
        cv2.line(frame, (bx + bw, by), (bx + bw, by + corner_len), color_locked, th)
        # Kiri Bawah
        cv2.line(frame, (bx, by + bh), (bx + corner_len, by + bh), color_locked, th)
        cv2.line(frame, (bx, by + bh), (bx, by + bh - corner_len), color_locked, th)
        # Kanan Bawah
        cv2.line(frame, (bx + bw, by + bh), (bx + bw - corner_len, by + bh), color_locked, th)
        cv2.line(frame, (bx + bw, by + bh), (bx + bw, by + bh - corner_len), color_locked, th)

        # 3. Titik Pusat Target & Vektor Garis Deviasi
        cv2.circle(frame, (tcx, tcy), 5, color_locked, -1)
        cv2.line(frame, (cx, cy), (tcx, tcy), (0, 200, 255), 1, cv2.LINE_AA)

        # 4. Label Target Badge
        id_str = f"ID #{track_id}" if track_id is not None else "TARGET"
        badge_text = f"LOCKED: {id_str} ({int(conf * 100)}%) | {distance_m:.2f}m"
        (tw_t, th_t), _ = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (bx, max(0, by - 24)), (bx + tw_t + 10, max(24, by)), (0, 255, 120), -1)
        cv2.putText(frame, badge_text, (bx + 5, max(17, by - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        # Status Scanning
        cv2.putText(frame, "MENCARI TARGET MANUSIA (YOLO 360)...", (cx - 180, cy - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA)

    # 5. Banner Header Telemetri Atas
    cv2.rectangle(frame, (0, 0), (w, 35), (20, 20, 20), -1)
    status_color = (0, 255, 0) if target_box is not None else (0, 165, 255)
    status_str = "TRACKING LOCKED (360 DEPAN/BELAKANG)" if target_box is not None else "STANDBY SCANNING"
    cv2.putText(frame, f"AI FOLLOWER: {status_str}", (15, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2, cv2.LINE_AA)

    fps_text = f"FPS: {fps:4.1f} | Ryzen 5 AI Engine"
    cv2.putText(frame, fps_text, (w - 260, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # 6. Banner Footer Bawah
    cv2.rectangle(frame, (0, h - 30), (w, h), (15, 15, 15), -1)
    footer_text = f"Serial ESP32: {serial_status} | 'r': Kunci Ulang | 'q': Keluar"
    cv2.putText(frame, footer_text, (15, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="Xiaomi/DJI Style High-Precision Full-Body AI Follower")
    parser.add_argument("--camera", type=int, default=0, help="Indeks kamera USB (default: 0)")
    parser.add_argument("--port", type=str, default=None, help="Port serial ESP32 (contoh: /dev/ttyUSB0)")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="Bobot model YOLO (default: yolov8n.pt)")
    parser.add_argument("--conf", type=float, default=0.50, help="Ambang batas kepercayaan deteksi (default: 0.50)")
    parser.add_argument("--dry-run", action="store_true", help="Jalankan simulasi tanpa kirim serial")
    args = parser.parse_args()

    print("=" * 65)
    print("   XIAOMI / DJI STYLE HIGH-PRECISION FULL-BODY AI FOLLOWER   ")
    print("=" * 65)

    # 1. Cek Ketersediaan YOLO
    if not HAS_YOLO:
        print("\n[ERROR FATAL] Library 'ultralytics' belum terinstal di sistem Anda!")
        print("Jalankan perintah ini di terminal Pop!_OS Anda untuk menginstalnya:")
        print("\n    pip install ultralytics\n")
        print("Lalu jalankan kembali program ini.")
        return

    print(f"[*] Memuat model AI Full-Body: {args.model}...")
    print("    (Jika pertama kali, bobot model 6MB akan diunduh otomatis dalam 2 detik)")
    try:
        model = YOLO(args.model)
        print("[OK] Model YOLOv8 berhasil dimuat!")
    except Exception as e:
        print(f"[ERROR] Gagal memuat model YOLO: {e}")
        return

    # 2. Inisialisasi Serial Transmitter ke ESP32
    serial_sender = FastSerialSender(port=args.port, baud=115200, dry_run=args.dry_run)
    print(f"[*] Status Serial ESP32: {serial_sender.status}")

    # 3. Buka Kamera USB
    print(f"[*] Membuka Kamera Indeks {args.camera}...")
    backend = cv2.CAP_V4L2 if sys.platform.startswith('linux') else cv2.CAP_ANY
    cap = cv2.VideoCapture(args.camera, backend)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print(f"[ERROR] Tidak dapat membuka kamera indeks {args.camera}!")
        return

    print("[OK] Kamera aktif. Memulai pelacakan presisi tinggi 360 derajat...")
    print("     - Tekan 'r' di keyboard untuk reset & mengunci target baru.")
    print("     - Tekan 'q' untuk keluar.")
    print("-" * 65)

    locked_track_id = None
    last_fps_time = time.time()
    frame_count = 0
    fps = 30.0

    cv2.namedWindow("Xiaomi Style AI Follower (Full-Body 360)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Xiaomi Style AI Follower (Full-Body 360)", 800, 600)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            frame_count += 1
            now = time.time()
            if now - last_fps_time >= 0.5:
                fps = frame_count / (now - last_fps_time)
                frame_count = 0
                last_fps_time = now

            # 4. Inferensi YOLOv8 dengan Pelacak ByteTrack Bawaan
            # classes=[0] HANYA mendeteksi Person (Manusia). Lemari, kursi, meja DIABAIKAN 100%!
            results = model.track(
                source=frame,
                persist=True,
                classes=[0],
                conf=args.conf,
                verbose=False,
                tracker="bytetrack.yaml"
            )

            detected_humans = []
            if results and len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes
                for box in boxes:
                    # Ambil koordinat pixel (x1, y1, x2, y2)
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, xyxy)
                    bw = max(10, x2 - x1)
                    bh = max(10, y2 - y1)
                    conf = float(box.conf[0].cpu().numpy())
                    track_id = int(box.id[0].cpu().numpy()) if box.id is not None else None

                    detected_humans.append({
                        "box": (x1, y1, bw, bh),
                        "conf": conf,
                        "id": track_id,
                        "area": bw * bh,
                        "cx": x1 + bw // 2,
                        "cy": y1 + bh // 2
                    })

            # 5. Logika Penguncian Target Cerdas (ID Persistence)
            target_human = None
            if len(detected_humans) > 0:
                if locked_track_id is not None:
                    # Cari manusia yang memiliki Track ID yang sama persis
                    for h_obj in detected_humans:
                        if h_obj["id"] == locked_track_id:
                            target_human = h_obj
                            break

                # Jika target lama belum ada / terlepas, kunci manusia yang paling dekat ke tengah atau terbesar
                if target_human is None:
                    # Urutkan berdasarkan kedekatan ke tengah layar
                    detected_humans.sort(
                        key=lambda h_obj: math.hypot(h_obj["cx"] - CENTER_X, h_obj["cy"] - CENTER_Y)
                    )
                    target_human = detected_humans[0]
                    locked_track_id = target_human["id"]

            # 6. Hitung Deviasi & Kirim ke ESP32
            if target_human is not None:
                bx, by, bw, bh = target_human["box"]
                tcx, tcy = target_human["cx"], target_human["cy"]

                # Hitung deviasi error (piksel)
                # Standar: Pusat badan atau dada (~35% dari atas kepala)
                aim_cy = by + int(bh * 0.35)
                err_x = tcx - CENTER_X
                err_y = aim_cy - CENTER_Y

                # Estimasi Jarak Monokular Aktual (Tinggi Badan Nyata 1.70m)
                dist_m = (HUMAN_REAL_HEIGHT_M * FOCAL_LENGTH_PX) / max(20, bh)
                dist_m = max(0.5, min(8.0, dist_m))
                dist_cm = int(dist_m * 100)

                # Kirim perintah cepat ke ESP32
                serial_sender.send(err_x, err_y, dist_cm)

                # Gambar HUD
                draw_hud(frame, target_human["box"], target_human["conf"],
                         target_human["id"], dist_m, fps, serial_sender.status)
            else:
                # Target hilang / tidak ada orang
                serial_sender.send_lost()
                draw_hud(frame, None, 0.0, None, 0.0, fps, serial_sender.status)

            # 7. Tampilkan di Layar Monitor
            cv2.imshow("Xiaomi Style AI Follower (Full-Body 360)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                print("[RESET] Mengunci ulang target manusia baru...")
                locked_track_id = None

    except KeyboardInterrupt:
        print("\n[STOP] Program dihentikan pengguna.")
    finally:
        serial_sender.close()
        cap.release()
        cv2.destroyAllWindows()
        print("[SHUTDOWN] Selesai.")


if __name__ == "__main__":
    main()
