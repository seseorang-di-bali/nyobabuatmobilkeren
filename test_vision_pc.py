#!/usr/bin/env python3
"""
test_vision_pc.py - Visual Target Tracker & Serial Sender (PC Test Drive)

Fitur:
- Resolusi 320x240 @ 30 FPS (sesuai spesifikasi STB Armbian)
- Mode A: Auto-Lock KCF Tracker (Hitung mundur 3 detik, kunci kotak tengah)
- Mode B: Face Detection Cascade (Otomatis deteksi wajah/orang tanpa countdown)
- Serial sender ke ESP32: "X:<err_x>,Y:<err_y>\n" atau "LOST\n"
- Failsafe Dry-Run: Tetap bisa dijalankan untuk uji visual jika ESP32 belum dicolok.
"""

import sys
import os
import time
import argparse
import glob
import cv2
import serial

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
CENTER_X = FRAME_WIDTH // 2   # 160
CENTER_Y = FRAME_HEIGHT // 2  # 120

def find_serial_port():
    """Mencari port serial ESP32 yang tersedia secara otomatis."""
    # Pola port untuk Mac & Linux
    ports = glob.glob("/dev/tty.usbserial*") + glob.glob("/dev/cu.usbserial*") + \
            glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    # Pola port untuk Windows (COM)
    if sys.platform.startswith('win'):
        ports = [f"COM{i+1}" for i in range(256)]
        
    for p in ports:
        try:
            s = serial.Serial(p, 115200, timeout=0.1)
            s.close()
            return p
        except (OSError, serial.SerialException):
            pass
    return None

def main():
    parser = argparse.ArgumentParser(description="PC / STB Test Drive Vision Tracker")
    parser.add_argument("--camera", type=int, default=0, help="ID Kamera (default: 0)")
    parser.add_argument("--port", type=str, default=None, help="Port Serial ESP32 (contoh: /dev/ttyUSB0 atau COM3)")
    parser.add_argument("--mode", type=str, choices=["kcf", "face"], default="face",
                        help="Mode tracking: 'face' (otomatis) atau 'kcf' (auto-lock kotak tengah)")
    parser.add_argument("--dry-run", action="store_true", help="Jalankan tanpa mengirim serial (hanya simulasi di layar)")
    parser.add_argument("--headless", action="store_true", help="Jalankan tanpa jendela GUI (cocok untuk STB Armbian via SSH)")
    args = parser.parse_args()

    # Cek apakah sistem memiliki Display GUI (X11/Wayland/HDMI)
    is_headless = args.headless or (os.name != 'nt' and sys.platform != 'darwin' and not os.environ.get("DISPLAY"))
    if is_headless:
        print("[INFO] Berjalan dalam mode HEADLESS (tanpa GUI window). Output telemetri via terminal.")

    # Inisialisasi Serial
    ser = None
    if not args.dry_run:
        port_to_use = args.port or find_serial_port()
        if port_to_use:
            try:
                ser = serial.Serial(port_to_use, 115200, timeout=0.05)
                time.sleep(2.0)  # Tunggu ESP32 reboot setelah serial connect
                print(f"[SERIAL] Terhubung ke ESP32 di port: {port_to_use}")
            except Exception as e:
                print(f"[WARN] Gagal membuka port {port_to_use}: {e}. Berjalan dalam mode DRY-RUN.")
        else:
            print("[INFO] ESP32 tidak terdeteksi. Berjalan otomatis dalam mode DRY-RUN (simulasi).")
    else:
        print("[INFO] Mode DRY-RUN diaktifkan oleh pengguna.")

    # Buka Kamera
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print(f"[ERROR] Tidak dapat membuka kamera dengan indeks {args.camera}!")
        return

    # Inisialisasi Detektor
    face_cascade = None
    if args.mode == "face":
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        face_cascade = cv2.CascadeClassifier(cascade_path)

    tracker = None
    tracking_active = False
    lock_countdown_start = time.time()
    countdown_duration = 3.0  # 3 detik untuk berdiri di tengah
    box_w, box_h = 100, 140
    init_box = ((FRAME_WIDTH - box_w) // 2, (FRAME_HEIGHT - box_h) // 2, box_w, box_h)

    print("\n" + "="*50)
    print(" VISION TRACKER PC TEST DRIVE")
    print(" - Tekan 'q' untuk keluar")
    print(" - Tekan 'r' untuk reset/kunci ulang target (Mode KCF)")
    print(" - Tekan 'm' untuk toggle mode (Face Cascade <-> KCF)")
    print("="*50 + "\n")

    prev_time = time.time()
    last_serial_send = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Gagal membaca frame dari kamera.")
                break

            # Frame di-resize untuk memastikan tepat 320x240
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            target_box = None
            status_text = "MENCARI TARGET"
            status_color = (0, 0, 255)

            # MODE A: FACE DETECTION
            if args.mode == "face":
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5, minSize=(30, 30))
                if len(faces) > 0:
                    # Ambil wajah terbesar (paling dekat dengan kamera)
                    faces = sorted(faces, key=lambda b: b[2] * b[3], reverse=True)
                    x, y, w, h = faces[0]
                    target_box = (x, y, w, h)
                    status_text = "FACE DETECTED"
                    status_color = (0, 255, 0)
                else:
                    status_text = "NO FACE FOUND"
                    status_color = (0, 0, 255)

            # MODE B: KCF TRACKER (SESUAI BLUEPRINT)
            elif args.mode == "kcf":
                now = time.time()
                if not tracking_active:
                    elapsed = now - lock_countdown_start
                    remaining = countdown_duration - elapsed
                    if remaining > 0:
                        status_text = f"LOCKING IN {remaining:.1f}s"
                        status_color = (0, 255, 255)
                        # Gambar kotak panduan di tengah
                        x, y, w, h = init_box
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    else:
                        # Buat OpenCV Tracker KCF
                        try:
                            tracker = cv2.TrackerKCF_create()
                        except AttributeError:
                            tracker = cv2.legacy.TrackerKCF_create()

                        tracker.init(frame, init_box)
                        tracking_active = True
                        print("[TRACKER] Target terkunci di kotak tengah!")
                else:
                    success, box = tracker.update(frame)
                    if success:
                        target_box = [int(v) for v in box]
                        status_text = "KCF TRACKING"
                        status_color = (0, 255, 0)
                    else:
                        status_text = "TARGET LOST"
                        status_color = (0, 0, 255)

            # Hitung Deviasi & Kirim Serial
            err_x = 0
            err_y = 0
            has_target = target_box is not None

            if has_target:
                tx, ty, tw, th = target_box
                cx = tx + tw // 2
                cy = ty + th // 2
                err_x = cx - CENTER_X
                err_y = cy - CENTER_Y

                # Visualisasi Target
                cv2.rectangle(frame, (tx, ty), (tx + tw, ty + th), status_color, 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                # Garis dari titik tengah kamera ke titik target
                cv2.line(frame, (CENTER_X, CENTER_Y), (cx, cy), (255, 255, 0), 1)

            # Kirim Serial setiap ~30ms (sinkronisasi framerate)
            if ser and (time.time() - last_serial_send > 0.03):
                try:
                    if has_target:
                        msg = f"X:{err_x},Y:{err_y}\n"
                    else:
                        msg = "LOST\n"
                    ser.write(msg.encode('utf-8'))
                    last_serial_send = time.time()
                except serial.SerialException as e:
                    print(f"[SERIAL ERROR] {e}")
                    ser = None

            # Render UI / Overlay
            # 1. Crosshair netral tengah kamera
            cv2.drawMarker(frame, (CENTER_X, CENTER_Y), (200, 200, 200),
                           markerType=cv2.MARKER_CROSS, markerSize=16, thickness=1)

            # 2. Deadzone box (zona tenang toleransi +/- 12px)
            cv2.rectangle(frame, (CENTER_X - 12, CENTER_Y - 12), (CENTER_X + 12, CENTER_Y + 12),
                          (100, 100, 100), 1)

            # 3. FPS & Status Telemetri
            fps = 1.0 / (time.time() - prev_time + 1e-6)
            prev_time = time.time()

            if not is_headless:
                cv2.putText(frame, f"FPS: {fps:.1f} | Mode: {args.mode.upper()}", (10, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                cv2.putText(frame, f"Status: {status_text}", (10, 34),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_color, 1)

                serial_status = "ONLINE" if ser else "DRY-RUN (OFFLINE)"
                cv2.putText(frame, f"Serial: {serial_status}", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0) if ser else (150, 150, 150), 1)

                if has_target:
                    cv2.putText(frame, f"Err X: {err_x:+d} px | Err Y: {err_y:+d} px", (10, FRAME_HEIGHT - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

                # Tampilkan Window GUI
                cv2.imshow("Vision Tracker - PC Test Drive (320x240)", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    print("[RESET] Mengatur ulang target...")
                    tracking_active = False
                    lock_countdown_start = time.time()
                elif key == ord('m'):
                    args.mode = "kcf" if args.mode == "face" else "face"
                    tracking_active = False
                    lock_countdown_start = time.time()
                    print(f"[MODE] Beralih ke mode: {args.mode.upper()}")
            else:
                # Mode Headless (Terminal Output) - Print periodik tiap ~0.25 detik
                if int(time.time() * 4) != int((time.time() - 0.03) * 4):
                    if has_target:
                        print(f"[{status_text}] FPS: {fps:.1f} | ErrX: {err_x:+4d} | ErrY: {err_y:+4d} | Serial: {'SENT' if ser else 'OFF'}")
                    else:
                        print(f"[{status_text}] FPS: {fps:.1f} | Mencari target... | Serial: {'LOST' if ser else 'OFF'}")
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[STOP] Program dihentikan oleh pengguna (Ctrl+C).")
    finally:
        if ser:
            ser.write(b"LOST\n")
            ser.close()
        cap.release()
        if not is_headless:
            cv2.destroyAllWindows()
        print("[EXIT] Program selesai.")

if __name__ == "__main__":
    main()
