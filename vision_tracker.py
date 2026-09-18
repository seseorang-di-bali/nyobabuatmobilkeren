#!/usr/bin/env python3
"""
vision_tracker.py - STB Armbian Visual Tracker & Serial Sender with Web HUD
Bagian dari Proyek Autonomous Vision-Guided Following Robot

Fitur Utama:
- Target Hardware: STB Armbian Linux (ZTE B860H / HG680P, Quad-Core Cortex-A53)
- Resolusi Input: 320x240 @ 30 FPS
- Mode: KCF (default), MOSSE (ultra-fast ~200 FPS), Face Cascade Detector
- Protokol Serial: "X:<err_x>,Y:<err_y>\\n" atau "LOST\\n" ke ESP32
- Headless Mode: Telemetri real-time di terminal SSH
- Web Streaming HUD: Akses live video & telemetry dari Mac/PC/HP di jaringan WiFi yang sama!
- Auto-Detect: Deteksi otomatis webcam USB (/dev/video1) dan serial ESP32 (/dev/ttyUSB0)
"""

import sys
import os
import time
import argparse
import glob
import socket
import threading
import logging
import subprocess
import json
import cv2
import numpy as np
import serial
from flask import Flask, Response, jsonify, render_template_string, request

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
CENTER_X = FRAME_WIDTH // 2   # 160
CENTER_Y = FRAME_HEIGHT // 2  # 120

# Kalibrasi Jarak Monokular (Logitech C170, HFOV ~48°, resolusi 320x240)
FOCAL_LENGTH_PX = 340.0

# Profil Target Tracking (Badan Belakang / Punggung & Baju / Badan Penuh)
TARGET_PROFILES = {
    "torso": {
        "name": "Badan Atas & Baju Punggung",
        "box": (80, 110),
        "real_h": 0.80,
        "label": "BAJU & PUNGGUNG"
    },
    "body": {
        "name": "Badan Penuh & Celana (Full Body)",
        "box": (95, 160),
        "real_h": 1.65,
        "label": "BADAN & CELANA"
    },
    "face": {
        "name": "Wajah Saja (Face)",
        "box": (70, 85),
        "real_h": 0.20,
        "label": "WAJAH"
    }
}

# Silencing logging dari werkzeug/flask agar terminal SSH tetap bersih
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

def get_local_ip():
    """Mendeteksi IP LAN STB di jaringan WiFi / Ethernet."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

def find_camera_index(requested_index=None):
    """
    Mencari index kamera USB yang aktif.
    Di STB Armbian, /dev/video0 seringkali adalah hardware video-codec SoC,
    sedangkan USB Webcam asli berada di /dev/video1 atau /dev/video2.
    """
    if requested_index is not None and requested_index >= 0:
        return requested_index, f"/dev/video{requested_index}"

    # Prioritas 1: Cek via /dev/v4l/by-id (perangkat USB nyata)
    usb_v4l = glob.glob("/dev/v4l/by-id/*")
    for link in sorted(usb_v4l):
        try:
            real_path = os.path.realpath(link)
            if "video" in real_path:
                idx = int(real_path.split("video")[-1])
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2 if sys.platform.startswith('linux') else cv2.CAP_ANY)
                if cap.isOpened():
                    ret, _ = cap.read()
                    cap.release()
                    if ret:
                        return idx, real_path
        except Exception:
            pass

    # Prioritas 2: Scan index kamera
    test_indices = [0, 1, 2] if sys.platform == 'darwin' else [1, 2, 3, 0]
    for idx in test_indices:
        try:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2 if sys.platform.startswith('linux') else cv2.CAP_ANY)
            if cap.isOpened():
                ret, _ = cap.read()
                cap.release()
                if ret:
                    cam_name = f"Camera {idx}" if sys.platform == 'darwin' else f"/dev/video{idx}"
                    return idx, cam_name
        except Exception:
            pass

    default_name = "Camera 0" if sys.platform == 'darwin' else "/dev/video0"
    return 0, default_name

def find_serial_port():
    """Mencari port serial ESP32 yang tersedia di Linux Armbian."""
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + \
            glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/tty.usbserial*")
    if sys.platform.startswith('win'):
        ports += [f"COM{i+1}" for i in range(256)]
        
    for p in ports:
        try:
            s = serial.Serial(p, 115200, timeout=0.1)
            s.close()
            return p
        except (OSError, serial.SerialException):
            pass
    return None

class AsyncSerialSender:
    """
    Transmitter Serial Asinkronus (Background Daemon Thread):
    Menjalankan transmisi data ke ESP32 secara non-blocking di thread terpisah.
    Menjamin video tracking loop di STB selalu berjalan stabil di 30-40 FPS
    tanpa pernah terhambat oleh USB buffer blocking atau jeda mikrokontroler.
    """
    def __init__(self, port=None, baud=115200, dry_run=False):
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self.ser = None
        self.lock = threading.Lock()
        self.latest_msg = "LOST\n"
        self.state_str = "DRY" if dry_run else "CONNECTING"
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def send(self, msg):
        with self.lock:
            self.latest_msg = msg

    def _worker(self):
        last_reconnect = 0
        while self.running:
            if self.dry_run:
                time.sleep(0.05)
                continue

            now = time.time()
            if self.ser is None:
                if now - last_reconnect > 2.0:
                    last_reconnect = now
                    p = self.port or find_serial_port()
                    if p:
                        try:
                            # write_timeout=0.01 & timeout=0.01 memastikan tidak akan pernah memblokir
                            self.ser = serial.Serial(p, self.baud, timeout=0.01, write_timeout=0.01)
                            self.state_str = "TX_OK"
                            with vision_state.lock:
                                vision_state.serial_state = "TX_OK"
                                vision_state.serial_port = p
                            print(f"[SERIAL] Asynchronous worker terhubung ke ESP32 pada: {p}")
                        except Exception:
                            self.ser = None
                            self.state_str = "DISCONNECTED"
                            with vision_state.lock:
                                vision_state.serial_state = "DISCONNECTED"
                time.sleep(0.05)
                continue

            with self.lock:
                msg = self.latest_msg

            try:
                # Bersihkan input buffer agar buffer serial Linux TTY tidak meluap
                if self.ser.in_waiting > 0:
                    self.ser.reset_input_buffer()

                self.ser.write(msg.encode('utf-8'))
                self.state_str = "TX_OK"
                with vision_state.lock:
                    vision_state.serial_state = "TX_OK"
            except Exception:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.state_str = "DISCONNECTED"
                with vision_state.lock:
                    vision_state.serial_state = "DISCONNECTED"

            time.sleep(0.03)

    def close(self):
        self.running = False
        if self.ser:
            try:
                self.ser.write(b"LOST\n")
                self.ser.close()
            except Exception:
                pass

def get_ai_face_detector():
    """
    Memuat model Neural Network OpenCV YuNet (ONNX, ~227KB):
    Mampu mendeteksi wajah & manusia pada jarak jauh (hingga 4-5 meter),
    pencahayaan minim / bayangan lampu plafon, dan berbagai sudut wajah,
    dengan kecepatan sangat tinggi (~5-9 ms di STB ARM CPU).
    """
    candidates = [
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "face_detection_yunet_2023mar.onnx"),
        "/usr/share/opencv4/face_detection_yunet_2023mar.onnx",
        "/etc/stb_vision/face_detection_yunet_2023mar.onnx",
    ]
    for c in candidates:
        if c and os.path.exists(c) and hasattr(cv2, "FaceDetectorYN"):
            try:
                detector = cv2.FaceDetectorYN.create(
                    model=c,
                    config="",
                    input_size=(FRAME_WIDTH, FRAME_HEIGHT),
                    score_threshold=0.55,
                    nms_threshold=0.3,
                    top_k=5000
                )
                print(f"[AI DETECTOR] Model YuNet Neural Network berhasil dimuat dari: {c}")
                return detector
            except Exception as e:
                print(f"[AI WARN] Gagal memuat YuNet ONNX: {e}")
    return None

def run_yunet_detection(detector, frame):
    """Menjalankan inferensi YuNet AI pada frame."""
    if detector is None:
        return []
    try:
        detector.setInputSize((frame.shape[1], frame.shape[0]))
        _, faces = detector.detect(frame)
        if faces is not None and len(faces) > 0:
            # Urutkan berdasarkan skor confidence * luas
            faces = sorted(faces, key=lambda f: float(f[14]) * (float(f[2]) * float(f[3])), reverse=True)
            res = []
            for f in faces:
                fx, fy, fw, fh, conf = int(f[0]), int(f[1]), int(f[2]), int(f[3]), float(f[14])
                if conf >= 0.48:
                    res.append((fx, fy, fw, fh, conf))
            return res
    except Exception:
        pass
    return []

def derive_body_box_from_face(fx, fy, fw, fh, profile_type="torso"):
    """
    Menghitung proporsi tubuh manusia presisi berdasarkan ukuran dan posisi wajah:
    - Menghasilkan kotak yang presisi pada jarak dekat maupun sangat jauh (3-5 meter).
    """
    fcx = fx + fw // 2
    if profile_type == "torso":
        cw = max(32, min(FRAME_WIDTH, int(fw * 2.1)))
        ch = max(42, min(FRAME_HEIGHT, int(fh * 2.7)))
        top_y = fy + int(fh * 0.75)
    elif profile_type == "body":
        cw = max(40, min(FRAME_WIDTH, int(fw * 2.4)))
        ch = max(70, min(FRAME_HEIGHT, int(fh * 5.5)))
        top_y = max(0, fy - int(fh * 0.15))
    else:  # face
        cw, ch = int(fw * 1.2), int(fh * 1.3)
        top_y = max(0, fy - int(fh * 0.1))

    left_x = max(0, min(FRAME_WIDTH - cw, fcx - cw // 2))
    top_y = max(0, min(FRAME_HEIGHT - ch, top_y))
    return (left_x, top_y, cw, ch)

def get_cascade_classifier():
    """Mencari dan memuat file XML Haar Cascade Wajah secara tangguh (fallback)."""
    candidates = [
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "haarcascade_frontalface_default.xml"),
        os.path.join(getattr(cv2, "data", type('', (), {'haarcascades': ''})()).haarcascades or '', "haarcascade_frontalface_default.xml"),
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
    ]
    for c in candidates:
        if c and os.path.exists(c):
            clf = cv2.CascadeClassifier(c)
            if not clf.empty():
                return clf
    return None

def enhance_dynamic_range(frame, clip_limit=2.5):
    """
    Fitur Anti-Silau / Wide Dynamic Range (WDR):
    Menerangi bayangan gelap pada tubuh/baju dan meredam cahaya lampu/jendela yang menyilaukan.
    Menggunakan CLAHE pada channel Luminance (LAB Color Space).
    """
    try:
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl, a, b))
        return cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    except Exception:
        return frame

def get_clothes_color_signature(frame, box):
    """
    Sistem Memori Pakaian Cerdas (Smart Dual-Zone Clothes Re-ID):
    Merekam profil warna 2 zona terpisah dalam kotak tubuh:
    1. Zona Atas (15% - 52% tinggi): Baju / Dada / Punggung Atas.
    2. Zona Bawah (52% - 88% tinggi): Baju Bawah / Pinggang / Celana.
    
    Fitur Anti-Salah Majikan & Anti-Silau:
    - 2D Hue-Saturation (16x12 bin): Fokus pada pigmen warna murni kain (kebal bayangan & lampu plafon).
    - 1D Value (8 bin): Menjaga perbedaan warna gelap vs terang.
    - Trimming Tepi (20% kiri & kanan): Hanya mengambil kain tubuh tengah, membuang latar tembok/lemari.
    """
    try:
        x, y, w, h = [int(v) for v in box]
        x = max(0, min(frame.shape[1] - 1, x))
        y = max(0, min(frame.shape[0] - 1, y))
        w = max(6, min(frame.shape[1] - x, w))
        h = max(8, min(frame.shape[0] - y, h))

        # Fokus pada bagian tengah kain (buang 20% margin kiri dan kanan agar warna tembok tidak tercampur)
        cx1 = x + int(w * 0.20)
        cx2 = x + int(w * 0.80)
        if cx2 <= cx1:
            cx1, cx2 = x, x + w

        # Zona 1: Baju Atas (dada / punggung)
        uy1 = y + int(h * 0.15)
        uy2 = y + int(h * 0.52)
        roi_upper = frame[uy1:uy2, cx1:cx2]

        # Zona 2: Bawah (pinggang / celana)
        ly1 = y + int(h * 0.52)
        ly2 = y + int(h * 0.88)
        roi_lower = frame[ly1:ly2, cx1:cx2]

        def compute_patch_hist(roi):
            if roi.size == 0:
                return None, None, 0.0, 0.0
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            # 2D HS: 16 bin Hue, 12 bin Saturation (warna kain murni)
            hist_hs = cv2.calcHist([hsv], [0, 1], None, [16, 12], [0, 180, 0, 256])
            cv2.normalize(hist_hs, hist_hs, alpha=1.0, norm_type=cv2.NORM_L1)
            # 1D V: 8 bin Value (tingkat gelap-terang)
            hist_v = cv2.calcHist([hsv], [2], None, [8], [0, 256])
            cv2.normalize(hist_v, hist_v, alpha=1.0, norm_type=cv2.NORM_L1)

            # Ekstraksi mean saturasi & variansi tekstur grayscale (0.02 ms, native C)
            mean_sat = float(np.mean(hsv[:, :, 1]))
            roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            std_dev = float(np.std(roi_gray))
            return hist_hs, hist_v, mean_sat, std_dev

        u_hs, u_v, u_sat, u_std = compute_patch_hist(roi_upper)
        l_hs, l_v, l_sat, l_std = compute_patch_hist(roi_lower)

        if u_hs is None and l_hs is None:
            return None

        return {
            "u_hs": u_hs,
            "u_v": u_v,
            "u_sat": u_sat,
            "u_std": u_std,
            "l_hs": l_hs,
            "l_v": l_v,
            "l_sat": l_sat,
            "l_std": l_std
        }
    except Exception:
        return None

def compare_clothes_color(frame, box, target_sig):
    """
    Memeriksa kecocokan pakaian secara Dual-Zone (Baju Atas + Celana):
    - Jika warna cerah: 75% bobot pada Hue-Saturation (warna kain asli) + 25% Value.
    - Proteksi Achromatic (Lampu redup / baju abu-abu): Menguji kesamaan tekstur kain
      agar tidak tertipu kasur, bantal, atau tembok abu-abu yang warnanya mirip.
    """
    if target_sig is None:
        return 1.0
    curr_sig = get_clothes_color_signature(frame, box)
    if curr_sig is None:
        return 0.5
    try:
        if not isinstance(target_sig, dict):
            return 0.7

        def compare_patch(t_hs, t_v, t_sat, t_std, c_hs, c_v, c_sat, c_std):
            if t_hs is None or c_hs is None:
                return 0.5
            s_hs = float(cv2.compareHist(t_hs, c_hs, cv2.HISTCMP_INTERSECT))
            s_v = float(cv2.compareHist(t_v, c_v, cv2.HISTCMP_INTERSECT)) if (t_v is not None and c_v is not None) else 0.5

            avg_sat = 0.5 * ((t_sat or 0.0) + (c_sat or 0.0))
            # Jika gambar minim saturasi warna (achromatic / abu-abu di ruangan redup):
            if avg_sat < 28.0:
                base_s = 0.40 * s_hs + 0.60 * s_v
                # Bandingkan tekstur kain dengan penalti lembut (jangan memotong skor drastis saat jarak berubah)
                if t_std is not None and c_std is not None:
                    diff_std = abs(t_std - c_std)
                    tex_factor = max(0.75, 1.0 - (diff_std / 80.0))
                    return base_s * tex_factor
                return base_s * 0.85
            else:
                return 0.75 * s_hs + 0.25 * s_v

        score_upper = compare_patch(target_sig.get("u_hs"), target_sig.get("u_v"),
                                    target_sig.get("u_sat"), target_sig.get("u_std"),
                                    curr_sig.get("u_hs"), curr_sig.get("u_v"),
                                    curr_sig.get("u_sat"), curr_sig.get("u_std"))
        score_lower = compare_patch(target_sig.get("l_hs"), target_sig.get("l_v"),
                                    target_sig.get("l_sat"), target_sig.get("l_std"),
                                    curr_sig.get("l_hs"), curr_sig.get("l_v"),
                                    curr_sig.get("l_sat"), curr_sig.get("l_std"))

        total_score = 0.60 * score_upper + 0.40 * score_lower
        return max(0.0, min(1.0, float(total_score)))
    except Exception:
        return 0.5

def update_clothes_color_signature(target_sig, curr_sig, alpha=0.96):
    """
    Memperbarui memori pakaian secara adaptif (Exponential Moving Average):
    Menyesuaikan sedikit perubahan warna saat berpindah lampu ruangan tanpa melupakan warna asli.
    """
    if target_sig is None or curr_sig is None:
        return target_sig
    try:
        updated = {}
        for key in ["u_hs", "u_v", "l_hs", "l_v"]:
            t_h = target_sig.get(key)
            c_h = curr_sig.get(key)
            if t_h is not None and c_h is not None:
                new_h = cv2.addWeighted(t_h, alpha, c_h, 1.0 - alpha, 0)
                cv2.normalize(new_h, new_h, alpha=1.0, norm_type=cv2.NORM_L1)
                updated[key] = new_h
            else:
                updated[key] = t_h

        for key in ["u_sat", "u_std", "l_sat", "l_std"]:
            t_val = target_sig.get(key)
            c_val = curr_sig.get(key)
            if t_val is not None and c_val is not None:
                updated[key] = alpha * t_val + (1.0 - alpha) * c_val
            else:
                updated[key] = t_val

        return updated
    except Exception:
        return target_sig

def compute_box_motion(frame_diff, box):
    """Menghitung intensitas pergerakan piksel di dalam kotak target (0.01 ms)."""
    if frame_diff is None or box is None:
        return 0.0
    try:
        x, y, w, h = [int(v) for v in box]
        x = max(0, min(FRAME_WIDTH - 1, x))
        y = max(0, min(FRAME_HEIGHT - 1, y))
        w = max(2, min(FRAME_WIDTH - x, w))
        h = max(2, min(FRAME_HEIGHT - y, h))
        roi = frame_diff[y:y+h, x:x+w]
        return float(np.mean(roi)) if roi.size > 0 else 0.0
    except Exception:
        return 0.0

def find_target_in_frame(frame, target_hist, base_box_w, base_box_h, yunet_detector=None, face_cascade=None, profile_type="torso", last_target_box=None, frame_diff=None):
    """
    Sistem Persistent Re-Identification (Re-ID) & Anti-Benda Mati:
    1. Prioritas 1: YuNet AI Neural Network Detector (~5-9 ms).
    2. Prioritas 2: Haar Cascade Fallback.
    3. Prioritas 3: Deteksi Gerakan Tubuh (Motion Blobs, Anti-Benda Mati, ~0.3 ms).
    4. Prioritas 4: Ultra-Fast Color Back-Projection.
    Menerapkan Spatial Continuity Gating dan Motion Priority (membedakan manusia vs benda mati).
    """
    if target_hist is None:
        return None, 0.0

    best_box = None
    best_score = 0.0

    def calculate_effective_score(cand_b, raw_s):
        eff_s = raw_s
        if last_target_box is not None:
            lx = last_target_box[0] + last_target_box[2] // 2
            ly = last_target_box[1] + last_target_box[3] // 2
            cx = cand_b[0] + cand_b[2] // 2
            cy = cand_b[1] + cand_b[3] // 2
            dist = ((cx - lx) ** 2 + (cy - ly) ** 2) ** 0.5
            prox_factor = max(0.85, 1.0 - (dist / 600.0))
            eff_s = eff_s * prox_factor

        # Motion Gating (Anti-Benda Mati):
        if frame_diff is not None:
            mot = compute_box_motion(frame_diff, cand_b)
            if mot > 2.0:
                # Memiliki pergerakan intrinsik (manusia aktif): beri dorongan prioritas +15%
                eff_s = min(1.0, eff_s * 1.15)
            elif mot < 0.8:
                # Benda mati total / tidak ada gerakan: kurangi prioritas -18%
                eff_s = eff_s * 0.82
        return eff_s

    # 1. Prioritas Utama: YuNet AI Neural Network Detector (~5-9 ms)
    if yunet_detector:
        ai_faces = run_yunet_detection(yunet_detector, frame)
        if len(ai_faces) > 0:
            for (fx, fy, fw, fh, conf) in ai_faces:
                cand_box = derive_body_box_from_face(fx, fy, fw, fh, profile_type)
                for dx in [-10, 0, 10]:
                    cbx = max(0, min(FRAME_WIDTH - cand_box[2], cand_box[0] + dx))
                    test_b = (cbx, cand_box[1], cand_box[2], cand_box[3])
                    s = compare_clothes_color(frame, test_b, target_hist)
                    eff_s = calculate_effective_score(test_b, s)
                    if eff_s > best_score:
                        best_score = eff_s
                        best_box = test_b

            if best_score >= 0.25:
                return best_box, max(0.40, best_score)

    # 2. Prioritas Kedua: Haar Cascade Fallback
    if face_cascade and not face_cascade.empty():
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=3, minSize=(20, 20))
            if len(faces) > 0:
                faces = sorted(faces, key=lambda b: b[2] * b[3], reverse=True)
                for (fx, fy, fw, fh) in faces[:2]:
                    cand_box = derive_body_box_from_face(fx, fy, fw, fh, profile_type)
                    for dx in [-10, 0, 10]:
                        cbx = max(0, min(FRAME_WIDTH - cand_box[2], cand_box[0] + dx))
                        test_b = (cbx, cand_box[1], cand_box[2], cand_box[3])
                        s = compare_clothes_color(frame, test_b, target_hist)
                        eff_s = calculate_effective_score(test_b, s)
                        if eff_s > best_score:
                            best_score = eff_s
                            best_box = test_b

                if best_score >= 0.28:
                    return best_box, max(0.35, best_score)
        except Exception:
            pass

    # 3. Prioritas Ketiga: Motion Blob Scanning (Anti-Benda Mati, ~0.3 ms)
    # Mencari objek yang aktif bergerak di frame dan mencocokkan kriteria pakaian majikan
    if frame_diff is not None:
        try:
            _, diff_thresh = cv2.threshold(frame_diff, 18, 255, cv2.THRESH_BINARY)
            diff_blur = cv2.boxFilter(diff_thresh, -1, (15, 15))
            contours, _ = cv2.findContours(diff_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                ca = cv2.contourArea(cnt)
                if ca > 400:  # Blob pergerakan manusia
                    cx, cy, cw, ch = cv2.boundingRect(cnt)
                    cand_w = max(base_box_w, min(FRAME_WIDTH, cw + 8))
                    cand_h = max(base_box_h, min(FRAME_HEIGHT, ch + 16))
                    bx = max(0, min(FRAME_WIDTH - cand_w, cx + cw // 2 - cand_w // 2))
                    by = max(0, min(FRAME_HEIGHT - cand_h, cy))
                    test_b = (bx, by, cand_w, cand_h)
                    s = compare_clothes_color(frame, test_b, target_hist)
                    eff_s = calculate_effective_score(test_b, s)
                    if eff_s > best_score:
                        best_score = eff_s
                        best_box = test_b
        except Exception:
            pass

    # 4. Prioritas Keempat: Ultra-Fast Color Back-Projection (~1-2 ms, OpenCV Native C++)
    # Menggantikan 440 sliding-window loop yang sebelumnya menghabiskan 1000ms dan menyebabkan 1 FPS.
    if target_hist and isinstance(target_hist, dict) and target_hist.get("u_hs") is not None:
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            prob_map = cv2.calcBackProject([hsv], [0, 1], target_hist["u_hs"], [0, 180, 0, 256], 255.0)
            ksize = 21
            prob_blur = cv2.boxFilter(prob_map, -1, (ksize, ksize))
            _, max_val, _, max_loc = cv2.minMaxLoc(prob_blur)

            if max_val > 10:  # Klaster warna pakaian ditemukan
                cx, cy = max_loc
                for s in [0.85, 1.0, 1.15]:
                    cw = max(32, min(FRAME_WIDTH - 10, int(base_box_w * s)))
                    ch = max(42, min(FRAME_HEIGHT - 10, int(base_box_h * s)))
                    bx = max(0, min(FRAME_WIDTH - cw, cx - cw // 2))
                    by = max(0, min(FRAME_HEIGHT - ch, cy - int(ch * 0.35)))
                    cand_box = (bx, by, cw, ch)
                    score = compare_clothes_color(frame, cand_box, target_hist)
                    eff_s = calculate_effective_score(cand_box, score)
                    if eff_s > best_score:
                        best_score = eff_s
                        best_box = cand_box
        except Exception:
            pass

    if best_score >= 0.30:
        return best_box, best_score

    return None, best_score

def configure_camera_hardware(cap, cam_device):
    """
    Mengoptimalkan setting UVC camera (Logitech C170, dll) di Linux:
    - Mengaktifkan backlight compensation agar punggung/badan tidak gelap saat backlight terang.
    - Mengaktifkan power line frequency anti-flicker 50Hz (standar listrik Indonesia).
    - Menyetel auto-exposure adaptif.
    """
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    except Exception:
        pass

    if sys.platform.startswith('linux') and cam_device and os.path.exists(cam_device):
        try:
            # 1. Backlight compensation: mencerahkan subjek di depan cahaya terang
            subprocess.run(["v4l2-ctl", "-d", cam_device, "--set-ctrl=backlight_compensation=1"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # 2. Anti-flicker 50Hz (lampu ruangan)
            subprocess.run(["v4l2-ctl", "-d", cam_device, "--set-ctrl=power_line_frequency=1"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # 3. Auto white-balance
            subprocess.run(["v4l2-ctl", "-d", cam_device, "--set-ctrl=white_balance_temperature_auto=1"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # 4. Boost saturasi hardware (Logitech C170 default 32 sangat pucat di ruangan remang; 64 mengembalikan warna RGB asli)
            subprocess.run(["v4l2-ctl", "-d", cam_device, "--set-ctrl=saturation=64"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # 5. Naikkan kontras hardware agar pakaian terpisah jelas dari dinding / kasur
            subprocess.run(["v4l2-ctl", "-d", cam_device, "--set-ctrl=contrast=38"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # 6. Naikkan ketajaman (sharpness)
            subprocess.run(["v4l2-ctl", "-d", cam_device, "--set-ctrl=sharpness=32"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # 7. Kunci frame rate hardware agar tidak anjlok ke 1-2 FPS di ruangan gelap
            subprocess.run(["v4l2-ctl", "-d", cam_device, "--set-ctrl=exposure_auto_priority=0"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"[CAMERA] Optimasi hardware v4l2 diterapkan pada {cam_device} (Backlight Comp=1, 50Hz, Saturation=64, Contrast=38, FPS Lock).")
        except Exception as e:
            print(f"[CAMERA INFO] v4l2-ctl opsional tidak dapat dijalankan: {e}")

def create_tracker(tracker_type):
    """Inisialisasi OpenCV tracker (CSRT, KCF, atau MOSSE)."""
    t_type = tracker_type.lower()
    if t_type == "csrt":
        if hasattr(cv2, "TrackerCSRT_create"):
            return cv2.TrackerCSRT_create()
        elif hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
            return cv2.legacy.TrackerCSRT_create()
    elif t_type == "mosse":
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerMOSSE_create"):
            return cv2.legacy.TrackerMOSSE_create()
        elif hasattr(cv2, "TrackerMOSSE_create"):
            return cv2.TrackerMOSSE_create()
    
    # Default KCF Tracker (Multi-channel HOG & Color Names)
    if hasattr(cv2, "TrackerKCF_create"):
        return cv2.TrackerKCF_create()
    elif hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerKCF_create"):
        return cv2.legacy.TrackerKCF_create()
    elif hasattr(cv2, "TrackerMIL_create"):
        return cv2.TrackerMIL_create()
    return None


class ThreadedCamera:
    """
    Dedicated background thread to continuously capture frames from USB webcam.
    Ensures zero buffer lag: cap.read() always returns the latest live frame,
    dropping stale buffered frames in the background.
    """
    def __init__(self, src=0, width=320, height=240, fps=30):
        backend = cv2.CAP_V4L2 if sys.platform.startswith('linux') else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(src, backend)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.ret = False
        self.frame = None
        if self.cap.isOpened():
            self.ret, self.frame = self.cap.read()

        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            if not self.cap.isOpened():
                time.sleep(0.05)
                continue
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.ret = ret
                    self.frame = frame
            else:
                time.sleep(0.01)

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ret, self.frame.copy()

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=0.3)
        self.cap.release()


class OnlineServoLearner:
    """
    Sistem Pembelajaran Gerakan Servo Mandiri Saat Berjalan (Ultra-Lightweight Online Learning):
    - Berjalan real-time di STB Armbian tanpa membebani CPU (kompleksitas O(1), <0.005 ms per frame).
    - Mengevaluasi performa pelacakan per episode (1.5 detik / 45 frame):
      * Menghitung Loss (Rata-rata error jarak piksel, frekuensi osilasi/overshoot, dan getaran jitter).
      * Menghitung Efisiensi Reward (10% - 99%).
    - Melakukan adaptasi gradien parameter (Kecepatan respon Kp & Kelembutan damping EMA).
    - Menyimpan parameter terbaik ke 'learned_servo_params.json' agar tetap diingat setelah STB restart.
    """
    def __init__(self, config_file="learned_servo_params.json"):
        self.config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config_file)
        self.iteration = 0
        self.best_loss = 999.0
        self.status = "MENUNGGU TARGET"
        self.metrics_str = "Loss: - | R: -"

        # Parameter default jika belum ada file terlatih
        self.learned_speed = 38
        self.learned_smooth = 72
        self.learned_deadzone = 12

        # Buffer pengumpul metrik per-episode
        self.frame_count = 0
        self.sum_abs_err = 0.0
        self.sum_d_err = 0.0
        self.prev_err = 0.0
        self.overshoot_count = 0
        self.last_sign = 0
        self.episode_start = time.time()
        self.is_trained = False

        self.load_learned_params()

    def load_learned_params(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, "r") as f:
                    data = json.load(f)
                    self.learned_speed = int(data.get("servo_speed_pct", 38))
                    self.learned_smooth = int(data.get("servo_smoothing_pct", 72))
                    self.learned_deadzone = int(data.get("deadzone", 12))
                    self.best_loss = float(data.get("best_loss", 999.0))
                    self.iteration = int(data.get("iteration", 0))
                    self.is_trained = bool(data.get("trained", False))
                    print(f"[AI LEARNER] Memuat parameter terlatih: Speed={self.learned_speed}%, Smooth={self.learned_smooth}%, Deadzone={self.learned_deadzone}px (Best Loss: {self.best_loss:.1f})")
        except Exception as e:
            print(f"[AI LEARNER] Info load config: {e}")

    def save_learned_params(self):
        try:
            data = {
                "servo_speed_pct": int(self.learned_speed),
                "servo_smoothing_pct": int(self.learned_smooth),
                "deadzone": int(self.learned_deadzone),
                "best_loss": float(round(self.best_loss, 2)),
                "iteration": int(self.iteration),
                "trained": True,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            with open(self.config_path, "w") as f:
                json.dump(data, f, indent=2)
            print(f"[AI LEARNER] Parameter gerakan terbaik disimpan ke {self.config_path}!")
        except Exception as e:
            print(f"[AI LEARNER] Gagal menyimpan parameter: {e}")

    def step(self, err_x, current_speed_pct, current_smooth_pct, is_auto_tune_enabled):
        """
        Dijalankan setiap frame (0.002 ms).
        Mengumpulkan data pergerakan dan memperbarui kebijakan servo setiap ~45 frame.
        """
        if not is_auto_tune_enabled:
            self.status = "MANUAL CONTROL"
            self.metrics_str = "AI Nonaktif"
            return current_speed_pct, current_smooth_pct, self.learned_deadzone, self.status, self.metrics_str

        now = time.time()
        abs_err = abs(err_x)
        d_err = abs(err_x - self.prev_err)
        curr_sign = 1 if err_x > 6 else (-1 if err_x < -6 else 0)

        # Deteksi pembalikan arah (overshoot / osilasi bolak-balik melintasi target)
        if self.last_sign != 0 and curr_sign != 0 and curr_sign != self.last_sign:
            self.overshoot_count += 1

        self.last_sign = curr_sign
        self.prev_err = err_x

        self.sum_abs_err += abs_err
        self.sum_d_err += d_err
        self.frame_count += 1

        # Evaluasi episode setiap ~45 frame (~1.5 detik berjalan)
        if self.frame_count >= 45 or (now - self.episode_start > 1.6):
            mae = self.sum_abs_err / max(1, self.frame_count)
            jitter = self.sum_d_err / max(1, self.frame_count)
            overshoots = self.overshoot_count

            # Fungsi Loss: Gabungan deviasi jarak, penalti osilasi/reog, dan getaran jitter
            loss = round(0.40 * mae + 15.0 * overshoots + 0.35 * jitter, 1)
            reward_pct = max(10, min(99, int(100 - loss)))
            self.iteration += 1

            new_speed = current_speed_pct
            new_smooth = current_smooth_pct
            new_deadzone = self.learned_deadzone

            # 1. Adaptasi Anti-Overshoot: Jika servo bergoyang bolak-balik melewati majikan
            if overshoots >= 2:
                new_speed = max(16, current_speed_pct - 4)
                new_smooth = min(88, current_smooth_pct + 3)
                action_desc = "REDAM OSILASI"

            # 2. Adaptasi Respons Lambat: Jika error masih besar tapi tidak ada osilasi sama sekali
            elif mae > 26.0 and overshoots <= 1:
                new_speed = min(68, current_speed_pct + 3)
                new_smooth = max(35, current_smooth_pct - 2)
                action_desc = "BOOST RESPON"

            # 3. Adaptasi Anti-Jitter Dekat: Jika target sudah di tengah tapi servo bergetar halus
            elif mae < 10.0 and jitter > 5.0:
                new_deadzone = min(18, new_deadzone + 1)
                action_desc = "DEADZONE +1px"

            # 4. Kinerja Optimal Tercapai
            else:
                action_desc = "PAS & STABIL"
                if loss < self.best_loss:
                    self.best_loss = loss
                    self.learned_speed = new_speed
                    self.learned_smooth = new_smooth
                    self.learned_deadzone = new_deadzone
                    self.save_learned_params()

            self.status = f"TRAINING #{self.iteration} | {action_desc}"
            self.metrics_str = f"Loss: {loss} | R: {reward_pct}%"

            # Reset akumulator untuk episode berikutnya
            self.frame_count = 0
            self.sum_abs_err = 0.0
            self.sum_d_err = 0.0
            self.overshoot_count = 0
            self.episode_start = now

            return new_speed, new_smooth, new_deadzone, self.status, self.metrics_str

        return current_speed_pct, current_smooth_pct, self.learned_deadzone, self.status, self.metrics_str

    def update(self, err_x, current_speed_pct, current_smooth_pct, is_auto_tune_enabled):
        spd, sm, dz, st, met = self.step(err_x, current_speed_pct, current_smooth_pct, is_auto_tune_enabled)
        return spd, sm, st

AdaptiveServoTuner = OnlineServoLearner


class VisionState:
    """State thread-safe untuk berbagi data antara visual tracker dan Web HUD."""
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.mode = "kcf"        # Default KCF (stabil, tahan warna & anti-silau)
        self.status = "INITIALIZING"
        self.fps = 0.0
        self.err_x = 0
        self.err_y = 0
        self.has_target = False
        self.serial_state = "DISCONNECTED"
        self.serial_port = "None"
        self.active_clients = 0
        self.trigger_reset = False
        self.change_mode_req = None
        self.mirror = False      # Default mirror OFF (orientasi nyata dunia robot)
        self.flip_v = False      # Flip vertikal (jika kamera terbalik atas-bawah)
        self.invert_pan = False  # Invert arah putaran servo Pan horizontal
        self.invert_tilt = False # Invert arah putaran servo Tilt vertikal
        self.servo_speed_pct = 38       # Kecepatan respon servo (default 38% halus)
        self.servo_smoothing_pct = 72   # Kelembutan EMA (default 72% sinematik)
        self.servo_deadzone = 12        # Deadzone pixel anti-jitter
        self.auto_tune = True           # AI Online Learning aktif
        self.auto_tune_status = "MENUNGGU TARGET"
        self.auto_tune_metrics = "Loss: - | R: -"
        self.target_profile = "torso"  # Default: Badan Atas & Baju
        self.change_profile_req = None
        self.distance_m = 0.0
        self.distance_status = "SEARCHING"
        self.anti_silau = True   # Default Anti-Silau WDR ON
        self.clothes_match_pct = 100
        self.select_target_req = None  # Tuple (x, y) dari klik Web HUD

vision_state = VisionState()

# Template HTML modern untuk Web Dashboard di Browser Mac
HTML_PAGE = """<!DOCTYPE html>
<html lang="id">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>STB Vision Tracker HUD</title>
    <style>
        :root {
            --bg-color: #0c0f14;
            --card-bg: #161b22;
            --border-color: #30363d;
            --accent-color: #58a6ff;
            --success-color: #3fb950;
            --warn-color: #d29922;
            --danger-color: #f85149;
            --text-main: #f0f6fc;
            --text-sub: #8b949e;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
        body { background-color: var(--bg-color); color: var(--text-main); min-height: 100vh; padding: 20px; display: flex; flex-direction: column; align-items: center; }
        .container { max-width: 800px; width: 100%; display: flex; flex-direction: column; gap: 20px; }
        
        header { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid var(--border-color); padding-bottom: 14px; }
        .title-area h1 { font-size: 1.35rem; display: flex; align-items: center; gap: 8px; font-weight: 700; letter-spacing: 0.5px; }
        .title-area p { font-size: 0.85rem; color: var(--text-sub); }
        .badge-live { display: inline-flex; align-items: center; gap: 6px; padding: 4px 10px; border-radius: 20px; background: rgba(63, 185, 80, 0.15); color: var(--success-color); font-size: 0.8rem; font-weight: 600; border: 1px solid rgba(63, 185, 80, 0.3); }
        .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--success-color); box-shadow: 0 0 8px var(--success-color); animation: pulse 1.8s infinite; }
        @keyframes pulse { 0%, 100% { opacity: 1; transform: scale(1); } 50% { opacity: 0.4; transform: scale(0.85); } }

        .viewport-card { background: var(--card-bg); border-radius: 12px; border: 1px solid var(--border-color); overflow: hidden; display: flex; flex-direction: column; align-items: center; box-shadow: 0 8px 24px rgba(0,0,0,0.5); }
        .video-wrapper { position: relative; width: 100%; display: flex; justify-content: center; background: #000; }
        .video-wrapper img { width: 100%; max-width: 640px; height: auto; aspect-ratio: 4/3; object-fit: contain; display: block; cursor: crosshair; }
        
        .stats-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; width: 100%; }
        .stat-card { background: var(--card-bg); border: 1px solid var(--border-color); border-radius: 10px; padding: 14px; display: flex; flex-direction: column; gap: 4px; }
        .stat-label { font-size: 0.75rem; color: var(--text-sub); text-transform: uppercase; font-weight: 600; }
        .stat-value { font-size: 1.25rem; font-weight: 700; font-family: "SF Mono", Menlo, Consolas, monospace; }
        
        .control-panel { background: var(--card-bg); border: 1px solid var(--border-color); border-radius: 10px; padding: 16px; display: flex; flex-wrap: wrap; gap: 12px; align-items: center; justify-content: space-between; }
        .btn-group { display: flex; gap: 10px; flex-wrap: wrap; }
        button { cursor: pointer; padding: 10px 18px; border-radius: 8px; border: 1px solid var(--border-color); font-size: 0.88rem; font-weight: 600; display: inline-flex; align-items: center; gap: 8px; transition: all 0.2s; }
        button:hover { filter: brightness(1.15); transform: translateY(-1px); }
        button:active { transform: translateY(0); }
        .btn-primary { background: #1f6feb; border-color: #388bfd; color: #fff; }
        .btn-secondary { background: #21262d; border-color: #363b42; color: #c9d1d9; }
        .btn-warning { background: #9e6a03; border-color: #bb8009; color: #fff; }

        .deviation-bar { width: 100%; height: 6px; background: #21262d; border-radius: 3px; position: relative; margin-top: 6px; overflow: hidden; }
        .deviation-thumb { height: 100%; width: 50%; background: var(--accent-color); transition: all 0.1s ease-out; }

        footer { text-align: center; font-size: 0.75rem; color: var(--text-sub); margin-top: 20px; }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <div class="title-area">
                <h1>🤖 STB ARMBIAN VISION TRACKER</h1>
                <p>Armbian Linux Visual Robot Navigation Engine</p>
            </div>
            <div class="badge-live">
                <span class="dot"></span>
                <span>LIVE FEED</span>
            </div>
        </header>

        <div class="viewport-card">
            <div class="video-wrapper">
                <img id="stream" src="/video_feed" alt="Camera Stream" title="Klik / Tap tubuh Anda pada video untuk mengunci target seketika!" />
            </div>
            <div style="background: rgba(88, 166, 255, 0.12); border-top: 1px solid rgba(88, 166, 255, 0.25); width: 100%; padding: 8px 14px; font-size: 0.80rem; color: #58a6ff; display: flex; align-items: center; justify-content: center; gap: 8px;">
                <span>🎯</span>
                <span><strong>Klik / Tap Tubuh Anda di Video:</strong> Langsung mengunci target seketika di posisi klik!</span>
            </div>
        </div>

        <div class="stats-grid">
            <div class="stat-card">
                <span class="stat-label">Status Target</span>
                <span id="target-status" class="stat-value" style="color: var(--warn-color);">INITIALIZING</span>
            </div>
            <div class="stat-card">
                <span class="stat-label">Mode Tracking</span>
                <span id="tracking-mode" class="stat-value" style="color: var(--accent-color);">KCF</span>
            </div>
            <div class="stat-card">
                <span class="stat-label">Kecocokan Baju</span>
                <span id="clothes-val" class="stat-value" style="color: var(--success-color);">100%</span>
                <span id="clothes-status" style="font-size: 0.72rem; color: var(--text-sub); font-weight: 600;">WARNA TERKUNCI</span>
            </div>
            <div class="stat-card">
                <span class="stat-label">Estimasi Jarak</span>
                <span id="dist-val" class="stat-value" style="color: #58a6ff;">-- m</span>
                <span id="dist-status" style="font-size: 0.72rem; color: var(--text-sub); font-weight: 600;">MENUNGGU TARGET</span>
            </div>
            <div class="stat-card">
                <span class="stat-label">Frame Rate (FPS)</span>
                <span id="fps-val" class="stat-value">0.0</span>
            </div>
            <div class="stat-card">
                <span class="stat-label">Serial ESP32</span>
                <span id="serial-val" class="stat-value" style="color: var(--text-sub);">DISCONNECTED</span>
            </div>
        </div>

        <div class="stats-grid">
            <div class="stat-card">
                <div style="display: flex; justify-content: space-between;">
                    <span class="stat-label">Deviasi X (Horizontal)</span>
                    <span id="err-x-val" style="font-family: monospace; font-size: 0.9rem; font-weight: bold;">0 px</span>
                </div>
                <div class="deviation-bar">
                    <div id="bar-x" class="deviation-thumb" style="width: 50%;"></div>
                </div>
            </div>
            <div class="stat-card">
                <div style="display: flex; justify-content: space-between;">
                    <span class="stat-label">Deviasi Y (Vertikal)</span>
                    <span id="err-y-val" style="font-family: monospace; font-size: 0.9rem; font-weight: bold;">0 px</span>
                </div>
                <div class="deviation-bar">
                    <div id="bar-y" class="deviation-thumb" style="width: 50%;"></div>
                </div>
            </div>
        </div>

        <div class="control-panel">
            <span style="font-size: 0.85rem; font-weight: 600; color: var(--text-sub);">PROFIL UKURAN TARGET (PUNGGUNG / BAJU / WAJAH):</span>
            <div class="btn-group">
                <button id="btn-prof-torso" class="btn-primary" onclick="setProfile('torso')">👕 Baju & Punggung (Torso)</button>
                <button id="btn-prof-body" class="btn-secondary" onclick="setProfile('body')">🧍 Badan & Celana (Full)</button>
                <button id="btn-prof-face" class="btn-secondary" onclick="setProfile('face')">😀 Wajah Saja</button>
            </div>
        </div>

        <div class="control-panel">
            <span style="font-size: 0.85rem; font-weight: 600; color: var(--text-sub);">KONTROL TRACKER:</span>
            <div class="btn-group">
                <button class="btn-warning" onclick="triggerReset()">🎯 Kunci Ulang (Re-Lock)</button>
                <button id="btn-mode-kcf" class="btn-primary" onclick="switchMode('kcf')">Mode KCF (Stabil)</button>
                <button id="btn-mode-mosse" class="btn-secondary" onclick="switchMode('mosse')">Mode MOSSE (Fast)</button>
                <button id="btn-mode-face" class="btn-secondary" onclick="switchMode('face')">Mode Wajah</button>
                <button id="btn-toggle-wdr" class="btn-primary" onclick="toggleAntiSilau()">☀️ Anti-Silau (WDR ON)</button>
                <button id="btn-toggle-mirror" class="btn-secondary" onclick="toggleMirror()">🪞 Mirror (Kiri/Kanan)</button>
                <button id="btn-toggle-flip-v" class="btn-secondary" onclick="toggleFlipV()">↕️ Flip Vertikal</button>
                <button id="btn-toggle-invert-pan" class="btn-secondary" onclick="toggleInvertPan()">🔁 Invert Pan (Kiri/Kanan)</button>
            </div>
        </div>

        <div class="control-panel">
            <span style="font-size: 0.85rem; font-weight: 600; color: var(--text-sub);">PENGATUR KECEPATAN & AI ONLINE LEARNING SERVO:</span>
            <div style="display: flex; gap: 15px; flex-wrap: wrap; align-items: center; margin-top: 8px;">
                <div style="flex: 1; min-width: 180px;">
                    <label style="font-size: 0.8rem; color: var(--text-sub);">Kecepatan: <span id="val-speed" style="color: var(--accent-color); font-weight: bold;">38%</span></label>
                    <input type="range" id="slider-speed" min="10" max="100" value="38" style="width: 100%; cursor: pointer;" oninput="changeServoSpeed(this.value)">
                </div>
                <div style="flex: 1; min-width: 180px;">
                    <label style="font-size: 0.8rem; color: var(--text-sub);">Kelembutan (Anti-Sentak): <span id="val-smooth" style="color: var(--success-color); font-weight: bold;">72%</span></label>
                    <input type="range" id="slider-smooth" min="10" max="90" value="72" style="width: 100%; cursor: pointer;" oninput="changeServoSmooth(this.value)">
                </div>
                <div>
                    <button id="btn-toggle-autotune" class="btn-primary" onclick="toggleAutoTune()">🧠 AI Learning: ON</button>
                </div>
            </div>
            <div style="font-size: 0.78rem; color: var(--text-sub); margin-top: 6px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px;">
                <div>Status AI Training: <span id="val-autotune-status" style="color: var(--accent-color); font-weight: 600;">MENUNGGU TARGET</span></div>
                <div>Performa: <span id="val-autotune-metrics" style="color: var(--success-color); font-weight: 600;">Loss: - | R: -</span></div>
            </div>
        </div>

        <footer>
            Autonomous Vision-Guided Robot &bull; Streamed from STB Armbian to Mac Local Browser
        </footer>
    </div>

    <script>
        let isDraggingSlider = false;

        function triggerReset() {
            fetch('/api/reset', { method: 'POST' })
                .then(r => r.json())
                .then(data => console.log('Reset triggered'));
        }

        function switchMode(m) {
            fetch('/api/mode/' + m, { method: 'POST' })
                .then(r => r.json())
                .then(data => console.log('Mode set to: ' + m));
        }

        function setProfile(p) {
            fetch('/api/profile/' + p, { method: 'POST' })
                .then(r => r.json())
                .then(data => console.log('Profile set to: ' + p));
        }

        function toggleMirror() {
            fetch('/api/toggle_mirror', { method: 'POST' })
                .then(r => r.json())
                .then(data => console.log('Mirror toggled to: ' + data.mirror));
        }

        function toggleFlipV() {
            fetch('/api/toggle_flip_v', { method: 'POST' })
                .then(r => r.json())
                .then(data => console.log('Flip-V toggled to: ' + data.flip_v));
        }

        function toggleAntiSilau() {
            fetch('/api/toggle_anti_silau', { method: 'POST' })
                .then(r => r.json())
                .then(data => console.log('Anti-Silau toggled to: ' + data.anti_silau));
        }

        function toggleInvertPan() {
            fetch('/api/toggle_invert_pan', { method: 'POST' })
                .then(r => r.json())
                .then(data => console.log('Invert Pan toggled to: ' + data.invert_pan));
        }

        function changeServoSpeed(val) {
            isDraggingSlider = true;
            document.getElementById('val-speed').innerText = val + '%';
            fetch('/api/servo_speed?val=' + val, { method: 'POST' })
                .finally(() => setTimeout(() => { isDraggingSlider = false; }, 800));
        }

        function changeServoSmooth(val) {
            isDraggingSlider = true;
            document.getElementById('val-smooth').innerText = val + '%';
            fetch('/api/servo_smooth?val=' + val, { method: 'POST' })
                .finally(() => setTimeout(() => { isDraggingSlider = false; }, 800));
        }

        function toggleAutoTune() {
            fetch('/api/toggle_autotune', { method: 'POST' })
                .then(r => r.json())
                .then(data => console.log('Auto-tune toggled to:', data.auto_tune));
        }

        function updateTelemetry() {
            fetch('/api/status')
                .then(res => res.json())
                .then(data => {
                    document.getElementById('fps-val').innerText = data.fps.toFixed(1);
                    document.getElementById('tracking-mode').innerText = data.mode.toUpperCase();
                    
                    const statusEl = document.getElementById('target-status');
                    statusEl.innerText = data.status;
                    if (data.status.includes('MISMATCH')) {
                        statusEl.style.color = 'var(--warn-color)';
                    } else if (data.status.includes('LOCKED') || data.status.includes('DETECTED')) {
                        statusEl.style.color = 'var(--success-color)';
                    } else if (data.status.includes('LOCKING') || data.status.includes('RELOCK')) {
                        statusEl.style.color = 'var(--accent-color)';
                    } else {
                        statusEl.style.color = 'var(--danger-color)';
                    }

                    // Update Kecocokan Warna Pakaian
                    const clothesEl = document.getElementById('clothes-val');
                    const clothesStatusEl = document.getElementById('clothes-status');
                    const matchPct = (data.clothes_match_pct !== undefined) ? data.clothes_match_pct : 100;
                    if (data.has_target) {
                        clothesEl.innerText = matchPct + '%';
                        if (matchPct >= 50) {
                            clothesEl.style.color = 'var(--success-color)';
                            clothesStatusEl.innerText = 'WARNA BAJU COCOK';
                            clothesStatusEl.style.color = 'var(--success-color)';
                        } else {
                            clothesEl.style.color = 'var(--warn-color)';
                            clothesStatusEl.innerText = 'WARNA BEDA / TERHALANG';
                            clothesStatusEl.style.color = 'var(--warn-color)';
                        }
                    } else if (data.status.includes('MISMATCH')) {
                        clothesEl.innerText = matchPct + '%';
                        clothesEl.style.color = 'var(--danger-color)';
                        clothesStatusEl.innerText = 'MENOLAK OBJEK LAIN';
                        clothesStatusEl.style.color = 'var(--danger-color)';
                    } else {
                        clothesEl.innerText = '--%';
                        clothesStatusEl.innerText = 'MENUNGGU TARGET';
                        clothesStatusEl.style.color = 'var(--text-sub)';
                    }

                    // Update Jarak Target
                    const distEl = document.getElementById('dist-val');
                    const distStatusEl = document.getElementById('dist-status');
                    if (data.has_target && data.distance_m > 0) {
                        distEl.innerText = data.distance_m.toFixed(2) + ' m';
                        distStatusEl.innerText = data.distance_status || 'TERKUNCI';
                        if (data.distance_status && data.distance_status.includes('IDEAL')) {
                            distStatusEl.style.color = 'var(--success-color)';
                            distEl.style.color = 'var(--success-color)';
                        } else if (data.distance_status && data.distance_status.includes('DEKAT')) {
                            distStatusEl.style.color = 'var(--danger-color)';
                            distEl.style.color = 'var(--danger-color)';
                        } else {
                            distStatusEl.style.color = 'var(--warn-color)';
                            distEl.style.color = 'var(--warn-color)';
                        }
                    } else {
                        distEl.innerText = '-- m';
                        distEl.style.color = 'var(--text-sub)';
                        distStatusEl.innerText = 'MENUNGGU TARGET';
                        distStatusEl.style.color = 'var(--text-sub)';
                    }

                    const serialEl = document.getElementById('serial-val');
                    serialEl.innerText = data.serial_state;
                    serialEl.style.color = (data.serial_state === 'TX_OK') ? 'var(--success-color)' : 'var(--warn-color)';

                    document.getElementById('err-x-val').innerText = (data.err_x >= 0 ? '+' : '') + data.err_x + ' px';
                    document.getElementById('err-y-val').innerText = (data.err_y >= 0 ? '+' : '') + data.err_y + ' px';

                    // Update Deviasi visual bar (-160 s/d +160 -> 0% s/d 100%)
                    const pctX = Math.min(100, Math.max(0, ((data.err_x + 160) / 320) * 100));
                    const pctY = Math.min(100, Math.max(0, ((data.err_y + 120) / 240) * 100));
                    document.getElementById('bar-x').style.width = pctX + '%';
                    document.getElementById('bar-y').style.width = pctY + '%';

                    // Highlight tombol profil aktif
                    ['body', 'torso', 'face'].forEach(p => {
                        const btn = document.getElementById('btn-prof-' + p);
                        if (btn) {
                            btn.className = (data.target_profile === p) ? 'btn-primary' : 'btn-secondary';
                        }
                    });

                    // Highlight tombol mode aktif
                    ['kcf', 'mosse', 'face'].forEach(m => {
                        const btn = document.getElementById('btn-mode-' + m);
                        if (btn) {
                            btn.className = (data.mode === m) ? 'btn-primary' : 'btn-secondary';
                        }
                    });

                    // Highlight tombol toggle mirror & flip-v & anti-silau & invert-pan
                    const btnMirror = document.getElementById('btn-toggle-mirror');
                    if (btnMirror) {
                        btnMirror.className = data.mirror ? 'btn-primary' : 'btn-secondary';
                    }
                    const btnFlipV = document.getElementById('btn-toggle-flip-v');
                    if (btnFlipV) {
                        btnFlipV.className = data.flip_v ? 'btn-primary' : 'btn-secondary';
                    }
                    const btnInvertPan = document.getElementById('btn-toggle-invert-pan');
                    if (btnInvertPan) {
                        btnInvertPan.className = data.invert_pan ? 'btn-primary' : 'btn-secondary';
                        btnInvertPan.innerText = data.invert_pan ? '🔁 Invert Pan (ON)' : '🔁 Invert Pan (OFF)';
                    }
                    const btnWdr = document.getElementById('btn-toggle-wdr');
                    if (btnWdr) {
                        btnWdr.className = data.anti_silau ? 'btn-primary' : 'btn-secondary';
                        btnWdr.innerText = data.anti_silau ? '☀️ Anti-Silau (WDR ON)' : '☀️ Anti-Silau (WDR OFF)';
                    }

                    // Update Kontrol Auto-Tune & Slider Servo
                    const statusAuto = document.getElementById('val-autotune-status');
                    if (statusAuto && data.auto_tune_status) {
                        statusAuto.innerText = data.auto_tune_status;
                    }
                    const metricsAuto = document.getElementById('val-autotune-metrics');
                    if (metricsAuto && data.auto_tune_metrics) {
                        metricsAuto.innerText = data.auto_tune_metrics;
                    }
                    const btnAuto = document.getElementById('btn-toggle-autotune');
                    if (btnAuto) {
                        btnAuto.className = data.auto_tune ? 'btn-primary' : 'btn-secondary';
                        btnAuto.innerText = data.auto_tune ? '🧠 AI Learning: ON' : '🧠 AI Learning: MANUAL';
                    }
                    if (!isDraggingSlider) {
                        const sSpeed = document.getElementById('slider-speed');
                        const sSmooth = document.getElementById('slider-smooth');
                        if (sSpeed && data.servo_speed_pct !== undefined) {
                            sSpeed.value = data.servo_speed_pct;
                            document.getElementById('val-speed').innerText = data.servo_speed_pct + '%';
                        }
                        if (sSmooth && data.servo_smoothing_pct !== undefined) {
                            sSmooth.value = data.servo_smoothing_pct;
                            document.getElementById('val-smooth').innerText = data.servo_smoothing_pct + '%';
                        }
                    }
                })
                .catch(e => console.error(e));
        }

        const streamImg = document.getElementById('stream');
        if (streamImg) {
            streamImg.addEventListener('click', function(e) {
                const rect = streamImg.getBoundingClientRect();
                const clickX = e.clientX - rect.left;
                const clickY = e.clientY - rect.top;
                const normX = Math.max(0, Math.min(1, clickX / rect.width));
                const normY = Math.max(0, Math.min(1, clickY / rect.height));
                const frameX = Math.round(normX * 320);
                const frameY = Math.round(normY * 240);
                fetch('/api/select_target?x=' + frameX + '&y=' + frameY, { method: 'POST' })
                    .then(r => r.json())
                    .then(data => console.log('Target selected at:', frameX, frameY))
                    .catch(err => console.error(err));
            });
        }

        setInterval(updateTelemetry, 250);
    </script>
</body>
</html>
"""

def create_app():
    """Membuat Flask app untuk Web HUD & MJPEG stream."""
    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template_string(HTML_PAGE)

    @app.route('/video_feed')
    def video_feed():
        def generate():
            with vision_state.lock:
                vision_state.active_clients += 1
            try:
                while True:
                    with vision_state.lock:
                        frame_bytes = vision_state.latest_jpeg
                    if frame_bytes is not None:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    time.sleep(0.04)  # ~25 FPS stream
            finally:
                with vision_state.lock:
                    vision_state.active_clients = max(0, vision_state.active_clients - 1)

        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/snapshot')
    def snapshot():
        """Mengambil satu frame snapshot terbaru dalam format JPEG."""
        with vision_state.lock:
            frame_bytes = vision_state.latest_jpeg
        if frame_bytes is not None:
            return Response(frame_bytes, mimetype='image/jpeg')
        return ("Frame belum tersedia. Pastikan kamera menyala.", 503)

    @app.route('/api/status')
    def api_status():
        with vision_state.lock:
            return jsonify({
                "mode": str(vision_state.mode),
                "target_profile": str(vision_state.target_profile),
                "status": str(vision_state.status),
                "fps": float(round(float(vision_state.fps), 1)),
                "err_x": int(vision_state.err_x),
                "err_y": int(vision_state.err_y),
                "has_target": bool(vision_state.has_target),
                "distance_m": float(round(float(vision_state.distance_m), 2)),
                "distance_cm": int(float(vision_state.distance_m) * 100),
                "distance_status": str(vision_state.distance_status),
                "serial_state": str(vision_state.serial_state),
                "serial_port": str(vision_state.serial_port),
                "mirror": bool(vision_state.mirror),
                "flip_v": bool(vision_state.flip_v),
                "invert_pan": bool(vision_state.invert_pan),
                "servo_speed_pct": int(vision_state.servo_speed_pct),
                "servo_smoothing_pct": int(vision_state.servo_smoothing_pct),
                "servo_deadzone": int(vision_state.servo_deadzone),
                "auto_tune": bool(vision_state.auto_tune),
                "auto_tune_status": str(vision_state.auto_tune_status),
                "auto_tune_metrics": str(vision_state.auto_tune_metrics),
                "anti_silau": bool(vision_state.anti_silau),
                "clothes_match_pct": int(vision_state.clothes_match_pct)
            })

    @app.route('/api/reset', methods=['POST', 'GET'])
    def api_reset():
        with vision_state.lock:
            vision_state.trigger_reset = True
        return jsonify({"status": "ok", "action": "reset_lock"})

    @app.route('/api/profile/<target_profile>', methods=['POST', 'GET'])
    def api_set_profile(target_profile):
        target_profile = target_profile.lower()
        if target_profile in TARGET_PROFILES:
            with vision_state.lock:
                vision_state.change_profile_req = target_profile
                vision_state.trigger_reset = True
            return jsonify({"status": "ok", "profile": target_profile})
        return jsonify({"status": "error", "message": "Profil tidak valid"}), 400

    @app.route('/api/toggle_mirror', methods=['POST', 'GET'])
    def api_toggle_mirror():
        with vision_state.lock:
            vision_state.mirror = not vision_state.mirror
            vision_state.trigger_reset = True
            new_val = vision_state.mirror
        return jsonify({"status": "ok", "mirror": new_val})

    @app.route('/api/toggle_flip_v', methods=['POST', 'GET'])
    def api_toggle_flip_v():
        with vision_state.lock:
            vision_state.flip_v = not vision_state.flip_v
            vision_state.trigger_reset = True
            new_val = vision_state.flip_v
        return jsonify({"status": "ok", "flip_v": new_val})

    @app.route('/api/toggle_invert_pan', methods=['POST', 'GET'])
    def api_toggle_invert_pan():
        with vision_state.lock:
            vision_state.invert_pan = not vision_state.invert_pan
            new_val = vision_state.invert_pan
        return jsonify({"status": "ok", "invert_pan": new_val})

    @app.route('/api/servo_speed', methods=['POST', 'GET'])
    def api_servo_speed():
        try:
            val = int(request.args.get('val', 40))
            val = max(10, min(100, val))
            with vision_state.lock:
                vision_state.servo_speed_pct = val
            return jsonify({"status": "ok", "servo_speed_pct": val})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 400

    @app.route('/api/servo_smooth', methods=['POST', 'GET'])
    def api_servo_smooth():
        try:
            val = int(request.args.get('val', 70))
            val = max(10, min(90, val))
            with vision_state.lock:
                vision_state.servo_smoothing_pct = val
            return jsonify({"status": "ok", "servo_smoothing_pct": val})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 400

    @app.route('/api/toggle_autotune', methods=['POST', 'GET'])
    def api_toggle_autotune():
        with vision_state.lock:
            vision_state.auto_tune = not vision_state.auto_tune
            val = vision_state.auto_tune
            vision_state.auto_tune_status = "AKTIF" if val else "MANUAL"
        return jsonify({"status": "ok", "auto_tune": val})

    @app.route('/api/toggle_anti_silau', methods=['POST', 'GET'])
    def api_toggle_anti_silau():
        with vision_state.lock:
            vision_state.anti_silau = not vision_state.anti_silau
            new_val = vision_state.anti_silau
        return jsonify({"status": "ok", "anti_silau": new_val})

    @app.route('/api/mode/<target_mode>', methods=['POST', 'GET'])
    def api_change_mode(target_mode):
        target_mode = target_mode.lower()
        if target_mode in ["kcf", "mosse", "face"]:
            with vision_state.lock:
                vision_state.change_mode_req = target_mode
            return jsonify({"status": "ok", "mode": target_mode})
        return jsonify({"status": "error", "message": "Mode tidak valid"}), 400

    @app.route('/api/select_target', methods=['POST', 'GET'])
    def api_select_target():
        try:
            x = int(request.args.get('x', 160))
            y = int(request.args.get('y', 120))
            with vision_state.lock:
                vision_state.select_target_req = (x, y)
            return jsonify({"status": "ok", "target": [x, y]})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 400

    return app

def main():
    parser = argparse.ArgumentParser(description="STB Armbian Vision Tracker with Web HUD")
    parser.add_argument("--camera", type=int, default=None, help="ID Kamera UVC (default: auto-detect)")
    parser.add_argument("--port", type=str, default=None, help="Port Serial ESP32 (contoh: /dev/ttyUSB0)")
    parser.add_argument("--mode", type=str, choices=["kcf", "mosse", "face"], default="kcf",
                        help="Mode tracking: 'kcf' (stabil, fitur warna & HOG, default), 'mosse' (ultra-fast), 'face' (deteksi wajah)")
    parser.add_argument("--profile", type=str, choices=["torso", "body", "face"], default="torso",
                        help="Profil target: 'torso' (baju & punggung, default), 'body' (seluruh badan & celana), 'face' (wajah)")
    parser.add_argument("--headless", action="store_true", help="Paksa mode tanpa GUI HDMI")
    parser.add_argument("--dry-run", action="store_true", help="Simulasi tanpa mengirim serial ke ESP32")
    parser.add_argument("--no-web", action="store_true", help="Nonaktifkan streaming Web HUD")
    parser.add_argument("--web-port", type=int, default=8080, help="Port Web HUD streaming (default: 8080)")
    parser.add_argument("--mirror", action="store_true", help="Aktifkan mirror horizontal (kamera selfie). Default OFF untuk robot.")
    parser.add_argument("--no-mirror", action="store_true", help="Nonaktifkan mirror horizontal (legacy flag)")
    parser.add_argument("--invert-pan", action="store_true", help="Balik arah putaran servo Pan horizontal (jika mekanik terbalik)")
    parser.add_argument("--invert-tilt", action="store_true", help="Balik arah putaran servo Tilt vertikal")
    parser.add_argument("--flip-v", action="store_true", help="Aktifkan flip vertikal (jika kamera terbalik atas-bawah)")
    parser.add_argument("--no-wdr", action="store_true", help="Nonaktifkan filter Anti-Silau (Wide Dynamic Range CLAHE)")
    args = parser.parse_args()

    vision_state.mirror = args.mirror and not args.no_mirror
    vision_state.flip_v = args.flip_v
    vision_state.invert_pan = args.invert_pan
    vision_state.invert_tilt = args.invert_tilt
    vision_state.target_profile = args.profile
    vision_state.mode = args.mode
    vision_state.anti_silau = not args.no_wdr

    # Cek ketersediaan GUI Display fisik (HDMI / X11)
    has_display = bool(os.environ.get("DISPLAY"))
    is_headless = args.headless or not has_display

    # Auto-detect camera index
    cam_index, cam_device = find_camera_index(args.camera)

    # Deteksi IP lokal STB untuk streaming ke Mac
    local_ip = get_local_ip()
    web_url = f"http://{local_ip}:{args.web_port}"

    prof_info = TARGET_PROFILES.get(args.profile, TARGET_PROFILES["torso"])

    print("=" * 60)
    print("      STB ARMBIAN VISION TRACKER (ROBOTICS ENGINE)     ")
    print(f" - Resolusi : {FRAME_WIDTH}x{FRAME_HEIGHT} @ 30 FPS")
    print(f" - Mode     : {args.mode.upper()} (Tracking Punggung & Warna Baju)")
    print(f" - Profil   : {prof_info['name']}")
    print(f" - Kamera   : Index {cam_index} ({cam_device})")
    print(f" - Jarak    : Monocular Vision Estimator Aktif")
    print(f" - Anti-Silau: {'ON (WDR CLAHE)' if vision_state.anti_silau else 'OFF'}")
    print(f" - InvertPan : {'ON (Dibalik)' if vision_state.invert_pan else 'OFF (Normal)'}")
    print(f" - Mirror   : {'ON (Horizontal)' if vision_state.mirror else 'OFF (Robot View)'}{' + [FLIP-V]' if vision_state.flip_v else ''}")
    print(f" - UI Mode  : {'HEADLESS (SSH Terminal)' if is_headless else 'DESKTOP GUI (HDMI)'}")
    if not args.no_web:
        print(f" - Web HUD  : \033[1;32m{web_url}\033[0m  <-- Buka di browser Mac!")
    print("=" * 60)

    # Jalankan Web Server di background daemon thread
    if not args.no_web:
        app = create_app()
        web_thread = threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=args.web_port, threaded=True, use_reloader=False),
            daemon=True
        )
        web_thread.start()
        print(f"[WEB] Server visualisasi aktif di port {args.web_port}")

    # Inisialisasi Serial Asinkronus ke ESP32 (Non-blocking background thread)
    serial_sender = AsyncSerialSender(port=args.port, baud=115200, dry_run=args.dry_run)
    vision_state.serial_state = serial_sender.state_str
    vision_state.serial_port = args.port or ("AUTO" if not args.dry_run else "DRY_RUN")
    vision_state.mode = args.mode

    # Inisialisasi Kamera UVC dengan Background Threading (Zero-Latency Buffer)
    cap = ThreadedCamera(cam_index, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=30)

    if not cap.isOpened():
        print(f"[ERROR] Tidak dapat membuka kamera pada index {cam_index} ({cam_device})!")
        if sys.platform == 'darwin':
            print("[MACOS TIPS] Izin kamera belum diberikan ke aplikasi Terminal / iTerm.")
            print("[MACOS TIPS] Buka: System Settings -> Privacy & Security -> Camera, lalu pastikan Terminal / iTerm diizinkan (ON).")
        else:
            print("[ERROR] Cek apakah webcam USB sudah terhubung dengan: ls -l /dev/video*")
        return

    # Optimasi Hardware UVC Camera (Backlight Compensation, Anti-Silau & Anti-Flicker)
    configure_camera_hardware(cap.cap, cam_device)

    # Inisialisasi Detektor Wajah (YuNet Neural Network + Haar Cascade Fallback)
    face_cascade = get_cascade_classifier()
    yunet_detector = get_ai_face_detector()

    # Parameter Tracking Bounding Box & Target Profile
    current_profile = args.profile
    prof_info = TARGET_PROFILES.get(current_profile, TARGET_PROFILES["torso"])
    box_w, box_h = prof_info["box"]
    init_box = ((FRAME_WIDTH - box_w) // 2, (FRAME_HEIGHT - box_h) // 2, box_w, box_h)
    active_lock_box = None
    tracker = None
    target_clothes_hist = None
    tracking_active = False
    countdown_start = None
    countdown_duration = 1.0
    human_last_seen = 0.0
    smooth_distance = 0.0
    dist_status = "SEARCHING"
    last_face_scan = 0
    frame_counter = 0
    mismatch_streak = 0
    static_streak = 0
    scanning_owner_start = None
    smooth_err_x = 0.0
    smooth_err_y = 0.0
    servo_learner = OnlineServoLearner()
    # Muat parameter yang sudah dipelajari sebelumnya jika ada
    with vision_state.lock:
        vision_state.servo_speed_pct = servo_learner.learned_speed
        vision_state.servo_smoothing_pct = servo_learner.learned_smooth
        vision_state.servo_deadzone = servo_learner.learned_deadzone

    prev_gray_frame = None

    print("[SYSTEM] Pipeline kamera aktif. Memulai pelacakan...\n")

    prev_time = time.time()
    last_serial_send = 0
    last_serial_reconnect = 0
    last_terminal_print = 0
    last_idle_encode = 0
    current_mode = args.mode

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            frame_counter += 1
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            with vision_state.lock:
                do_mirror = vision_state.mirror
                do_flip_v = vision_state.flip_v
                do_anti_silau = vision_state.anti_silau
                do_invert_pan = vision_state.invert_pan
                do_invert_tilt = vision_state.invert_tilt

            # Mirror horizontal (kamera selfie / pembalik gambar) & Flip vertikal
            if do_mirror and do_flip_v:
                frame = cv2.flip(frame, -1)
            elif do_mirror:
                frame = cv2.flip(frame, 1)
            elif do_flip_v:
                frame = cv2.flip(frame, 0)

            # Optimasi Anti-Silau / Wide Dynamic Range (WDR CLAHE)
            if do_anti_silau:
                frame = enhance_dynamic_range(frame)

            # Komputasi Frame Difference Ultra-Cepat (~0.08 ms) untuk Anti-Benda Mati & Motion Detection
            curr_gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_diff = cv2.absdiff(curr_gray_frame, prev_gray_frame) if prev_gray_frame is not None else None
            prev_gray_frame = curr_gray_frame

            annotated_frame = frame.copy()
            target_box = None
            status_text = "SEARCHING"
            status_color = (0, 0, 255)

            # Cek perintah dari Web HUD / remote control
            with vision_state.lock:
                if vision_state.trigger_reset:
                    tracking_active = False
                    target_clothes_hist = None
                    mismatch_streak = 0
                    static_streak = 0
                    countdown_duration = 1.0
                    countdown_start = None
                    scanning_owner_start = None
                    active_lock_box = None
                    human_last_seen = 0.0
                    vision_state.clothes_match_pct = 100
                    vision_state.trigger_reset = False
                    print("[CONTROL] Reset pelacakan dari Web HUD diterima.")

                if vision_state.change_profile_req:
                    current_profile = vision_state.change_profile_req
                    vision_state.target_profile = current_profile
                    vision_state.change_profile_req = None
                    prof_info = TARGET_PROFILES.get(current_profile, TARGET_PROFILES["torso"])
                    box_w, box_h = prof_info["box"]
                    init_box = ((FRAME_WIDTH - box_w) // 2, (FRAME_HEIGHT - box_h) // 2, box_w, box_h)
                    active_lock_box = None
                    tracking_active = False
                    target_clothes_hist = None
                    mismatch_streak = 0
                    static_streak = 0
                    countdown_duration = 1.0
                    countdown_start = None
                    scanning_owner_start = None
                    human_last_seen = 0.0
                    vision_state.clothes_match_pct = 100
                    print(f"[CONTROL] Profil target diubah ke '{prof_info['name']}'.")

                if vision_state.change_mode_req:
                    current_mode = vision_state.change_mode_req
                    vision_state.mode = current_mode
                    vision_state.change_mode_req = None
                    tracking_active = False
                    target_clothes_hist = None
                    mismatch_streak = 0
                    static_streak = 0
                    countdown_duration = 1.0
                    countdown_start = None
                    scanning_owner_start = None
                    active_lock_box = None
                    human_last_seen = 0.0
                    vision_state.clothes_match_pct = 100
                    print(f"[CONTROL] Ganti mode ke '{current_mode.upper()}' dari Web HUD.")

            # Cek perintah Click-to-Track manual dari Web HUD
            with vision_state.lock:
                req_click = vision_state.select_target_req
                vision_state.select_target_req = None

            if req_click is not None:
                sel_x, sel_y = req_click
                prof_info = TARGET_PROFILES.get(current_profile, TARGET_PROFILES["torso"])
                clicked_box = None

                # 1. Cocokkan dengan orang yang dideteksi YuNet AI di dekat titik klik
                ai_faces = run_yunet_detection(yunet_detector, frame) if yunet_detector else []
                for (fx, fy, fw, fh, conf) in ai_faces:
                    p_box = derive_body_box_from_face(fx, fy, fw, fh, current_profile)
                    bx, by, bw, bh = p_box
                    if (bx - 20 <= sel_x <= bx + bw + 20) and (by - 20 <= sel_y <= by + bh + 20):
                        clicked_box = p_box
                        break

                # 2. Fallback Haar cascade
                if clicked_box is None and face_cascade:
                    try:
                        gray_c = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        faces_c = face_cascade.detectMultiScale(gray_c, scaleFactor=1.2, minNeighbors=3, minSize=(20, 20))
                        for (fx, fy, fw, fh) in faces_c:
                            p_box = derive_body_box_from_face(fx, fy, fw, fh, current_profile)
                            bx, by, bw, bh = p_box
                            if (bx - 20 <= sel_x <= bx + bw + 20) and (by - 20 <= sel_y <= by + bh + 20):
                                clicked_box = p_box
                                break
                    except Exception:
                        pass

                # 3. Jika diklik pada badan/baju yang membelakangi kamera (tampak punggung):
                if clicked_box is None:
                    base_w, base_h = prof_info["box"]
                    scale = 0.60 if sel_y > 140 else 0.80
                    cw = max(32, min(FRAME_WIDTH, int(base_w * scale)))
                    ch = max(42, min(FRAME_HEIGHT, int(base_h * scale)))
                    nx = max(0, min(FRAME_WIDTH - cw, sel_x - cw // 2))
                    ny = max(0, min(FRAME_HEIGHT - ch, sel_y - ch // 2))
                    clicked_box = (nx, ny, cw, ch)

                tracker = create_tracker(current_mode)
                if tracker:
                    tracker.init(frame, clicked_box)
                    target_clothes_hist = get_clothes_color_signature(frame, clicked_box)
                    tracking_active = True
                    target_box = list(clicked_box)
                    mismatch_streak = 0
                    static_streak = 0
                    scanning_owner_start = None
                    countdown_start = None
                    active_lock_box = None
                    with vision_state.lock:
                        vision_state.clothes_match_pct = 100
                    smooth_distance = (prof_info["real_h"] * FOCAL_LENGTH_PX) / max(10, clicked_box[3])
                    status_text = "LOCKED_CLICK"
                    status_color = (0, 255, 0)
                    print(f"[CLICK-TO-TRACK] Target berhasil dikunci langsung pada: {clicked_box}")

            # --- METODE 1: TRACKER DENGAN AUTO-LOCK & PERSISTENT RE-ID ---
            if current_mode in ["kcf", "mosse"]:
                now = time.time()
                prof_info = TARGET_PROFILES.get(current_profile, TARGET_PROFILES["torso"])
                box_w, box_h = prof_info["box"]

                if not tracking_active:
                    # KASUS A: SUDAH ADA MEMORI WARNA/BENTUK PEMILIK (Persistent Re-ID)
                    # Memori pemilik disimpan permanen! Terus pantau frame dan kunci kembali saat pemilik bergerak/terlihat
                    if target_clothes_hist is not None:
                        if scanning_owner_start is None:
                            scanning_owner_start = now

                        re_box, re_score = find_target_in_frame(frame, target_clothes_hist, box_w, box_h,
                                                               yunet_detector=yunet_detector,
                                                               face_cascade=face_cascade,
                                                               profile_type=current_profile,
                                                               frame_diff=frame_diff)
                        if re_box and re_score >= 0.30:
                            tracker = create_tracker(current_mode)
                            if tracker:
                                tracker.init(frame, re_box)
                                tracking_active = True
                                target_box = list(re_box)
                                mismatch_streak = 0
                                static_streak = 0
                                scanning_owner_start = None
                                initial_th = re_box[3]
                                smooth_distance = (prof_info["real_h"] * FOCAL_LENGTH_PX) / max(10, initial_th)
                                status_text = f"LOCKED_TRACKING ({int(re_score * 100)}%)"
                                status_color = (0, 255, 0)
                                print(f"[RE-SNAP] Pemilik terdeteksi ({int(re_score * 100)}% match)! Mengunci target kembali...")
                        else:
                            # Memori pemilik disimpan secara permanen (tidak ada timeout 2 detik untuk lupa majikan!)
                            status_text = "SCANNING_OWNER"
                            status_color = (0, 165, 255)
                            cx_box = ((FRAME_WIDTH - box_w) // 2, (FRAME_HEIGHT - box_h) // 2, box_w, box_h)
                            cv2.rectangle(annotated_frame, (cx_box[0], cx_box[1]),
                                          (cx_box[0] + cx_box[2], cx_box[1] + cx_box[3]), (0, 165, 255), 1)
                            scan_elapsed = now - scanning_owner_start
                            cv2.putText(annotated_frame, f"MEMORI AKTIF: MENCARI MAJIKAN ({scan_elapsed:.1f}s)...", (10, FRAME_HEIGHT - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 165, 255), 1)

                    # KASUS B: BELUM ADA TEMPLATE DI MEMORI (Awal Start atau User Klik Reset)
                    else:
                        center_box = ((FRAME_WIDTH - box_w) // 2, (FRAME_HEIGHT - box_h) // 2, box_w, box_h)

                        # Smart Human Detection: Scan setiap 0.06 detik (~16 fps) untuk mendeteksi manusia
                        if now - last_face_scan > 0.06:
                            last_face_scan = now
                            found_human = False
                            if yunet_detector:
                                ai_faces = run_yunet_detection(yunet_detector, frame)
                                if len(ai_faces) > 0:
                                    fx, fy, fw, fh, _ = ai_faces[0]
                                    active_lock_box = derive_body_box_from_face(fx, fy, fw, fh, current_profile)
                                    human_last_seen = now
                                    found_human = True

                            if not found_human and face_cascade:
                                try:
                                    gray_snap = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                                    faces_found = face_cascade.detectMultiScale(gray_snap, scaleFactor=1.2, minNeighbors=3, minSize=(20, 20))
                                    if len(faces_found) > 0:
                                        faces_found = sorted(faces_found, key=lambda b: b[2] * b[3], reverse=True)
                                        fx, fy, fw, fh = faces_found[0]
                                        active_lock_box = derive_body_box_from_face(fx, fy, fw, fh, current_profile)
                                        human_last_seen = now
                                        found_human = True
                                except Exception:
                                    pass

                            # Fallback Jarak Dekat (Close-up Torso):
                            # Jika kepala terpotong di atas frame, periksa apakah kotak tengah diisi oleh tubuh manusia
                            if not found_human:
                                try:
                                    cx, cy, cw, ch = center_box
                                    c_roi = frame[cy:cy+ch, cx:cx+cw]
                                    if c_roi.size > 0:
                                        c_gray = cv2.cvtColor(c_roi, cv2.COLOR_BGR2GRAY)
                                        c_std = float(np.std(c_gray))
                                        # Pakaian dan badan manusia memiliki variansi tekstur yang nyata (> 18.0)
                                        if c_std > 18.0:
                                            active_lock_box = center_box
                                            human_last_seen = now
                                            found_human = True
                                except Exception:
                                    pass

                        # Manusia dianggap aktif jika terdeteksi dalam 0.6 detik terakhir (toleransi flicker scan)
                        human_present = (now - human_last_seen < 0.6) and (active_lock_box is not None)

                        if not human_present:
                            # TIDAK ADA MANUSIA: Jangan pernah hitung mundur dan jangan mengunci pintu/tembok kosong!
                            countdown_start = None
                            status_text = "MENUNGGU_MAJIKAN"
                            status_color = (0, 200, 255)
                            cx, cy, cw, ch = center_box

                            # Gambar kotak panduan standby di tengah
                            cv2.rectangle(annotated_frame, (cx, cy), (cx + cw, cy + ch), (0, 200, 255), 1)
                            cv2.putText(annotated_frame, "BERDIRI DI DEPAN KAMERA ATAU KLIK VIDEO", (10, FRAME_HEIGHT - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1)
                            cv2.putText(annotated_frame, "STANDBY: MENUNGGU MAJIKAN...", (cx, max(14, cy - 8)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1)
                        else:
                            # ADA MANUSIA TERDETEKSI: Jalankan hitung mundur 1 detik untuk merekam target
                            if countdown_start is None:
                                countdown_start = now

                            elapsed = now - countdown_start
                            remaining = max(0.0, countdown_duration - elapsed)
                            x, y, w, h = active_lock_box

                            if remaining > 0:
                                status_text = f"LOCKING_IN_{remaining:.1f}S"
                                status_color = (0, 255, 255)

                                # Gambar Kotak Target Countdown yang menempel di manusia
                                cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                                cv2.putText(annotated_frame, f"REKAM TARGET {prof_info['label']}: {remaining:.1f}s", (x, max(12, y - 8)),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
                                cv2.putText(annotated_frame, "MANUSIA TERDETEKSI! DIAM SEBENTAR UNTUK MEREKAM", (10, FRAME_HEIGHT - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1)
                            else:
                                tracker = create_tracker(current_mode)
                                if tracker:
                                    tracker.init(frame, active_lock_box)
                                    target_clothes_hist = get_clothes_color_signature(frame, active_lock_box)
                                    tracking_active = True
                                    target_box = list(active_lock_box)
                                    mismatch_streak = 0
                                    countdown_start = None
                                    scanning_owner_start = None
                                    with vision_state.lock:
                                        vision_state.clothes_match_pct = 100
                                    initial_th = active_lock_box[3]
                                    smooth_distance = (prof_info["real_h"] * FOCAL_LENGTH_PX) / max(10, initial_th)
                                    status_text = "LOCKED_TRACKING (100%)"
                                    status_color = (0, 255, 0)
                                    print(f"[TRACKER] Target manusia terkunci & disimpan di memori ({prof_info['name']}) via {current_mode.upper()}!")
                                else:
                                    countdown_start = None
                                    print(f"[ERROR] Gagal membuat tracker {current_mode}!")

                else:
                    success, box = tracker.update(frame)
                    if success:
                        target_box = [int(v) for v in box]
                        bx, by, bw, bh = target_box

                        # 1. AI Anchor & Dynamic Scale Correction (Setiap 4 frame):
                        # Menggunakan YuNet Neural Net untuk menjaga kotak tetap melekat di tengah badan,
                        # mencegah kotak bergeser ke bahu saat bergerak cepat / ngereog,
                        # dan menyesuaikan ukuran kotak secara presisi saat pengguna menjauh (3-5m) atau mendekat.
                        ai_reanchored = False
                        if yunet_detector and (frame_counter % 4 == 0):
                            ai_faces = run_yunet_detection(yunet_detector, frame)
                            for (fx, fy, fw, fh, conf) in ai_faces:
                                ideal_box = derive_body_box_from_face(fx, fy, fw, fh, current_profile)
                                ix, iy, iw, ih = ideal_box
                                fcx = fx + fw // 2
                                tcx = bx + bw // 2
                                # Wajah sejajar horizontal dengan target
                                if abs(fcx - tcx) < max(bw, iw) * 0.85:
                                    drift_x = abs(tcx - (ix + iw // 2))
                                    scale_diff = abs(bw - iw) + abs(bh - ih)
                                    if drift_x > 8 or scale_diff > 12:
                                        new_w = max(30, min(FRAME_WIDTH, int(0.45 * bw + 0.55 * iw)))
                                        new_h = max(40, min(FRAME_HEIGHT, int(0.45 * bh + 0.55 * ih)))
                                        new_x = max(0, min(FRAME_WIDTH - new_w, int(0.45 * bx + 0.55 * ix)))
                                        new_y = max(0, min(FRAME_HEIGHT - new_h, int(0.45 * by + 0.55 * iy)))
                                        target_box = [new_x, new_y, new_w, new_h]
                                        bx, by, bw, bh = target_box
                                        tracker = create_tracker(current_mode)
                                        if tracker:
                                            tracker.init(frame, tuple(target_box))
                                    ai_reanchored = True
                                    break

                        # 2. Anti-Bahu Drift via Color Histogram Symmetry:
                        # Jika wajah tidak terlihat (misal tampak punggung), gunakan histogram warna
                        # untuk memastikan kotak selalu berada di tengah baju, bukan di pinggir bahu
                        if not ai_reanchored:
                            raw_score = compare_clothes_color(frame, target_box, target_clothes_hist)
                            offsets = [-12, -6, 6, 12]
                            best_dx = 0
                            best_s = raw_score
                            for dx in offsets:
                                tx = max(0, min(FRAME_WIDTH - bw, bx + dx))
                                s = compare_clothes_color(frame, (tx, by, bw, bh), target_clothes_hist)
                                if s > best_s:
                                    best_s = s
                                    best_dx = dx

                            if best_dx != 0 and (best_s > raw_score + 0.03):
                                new_x = max(0, min(FRAME_WIDTH - bw, bx + best_dx))
                                target_box[0] = new_x
                                bx = new_x
                                match_score = best_s
                                if abs(best_dx) >= 10:
                                    tracker = create_tracker(current_mode)
                                    if tracker:
                                        tracker.init(frame, tuple(target_box))
                            else:
                                match_score = raw_score
                        else:
                            match_score = compare_clothes_color(frame, target_box, target_clothes_hist)

                        clothes_pct = int(match_score * 100)
                        with vision_state.lock:
                            vision_state.clothes_match_pct = clothes_pct

                        # 3. Model Adaptation (EMA):
                        # Jika kecocokan sangat tinggi (>= 65%), adaptasikan sedikit variasi pencahayaan
                        if match_score >= 0.65:
                            curr_hist = get_clothes_color_signature(frame, target_box)
                            if curr_hist is not None and target_clothes_hist is not None:
                                target_clothes_hist = update_clothes_color_signature(target_clothes_hist, curr_hist, alpha=0.96)

                        # 4. Anti-Benda Mati (Static Object Rejection Filter):
                        # Memeriksa intensitas gerakan target. Jika target mati/diam (misal dinding/kursi),
                        # otomatis cari apakah ada majikan yang bergerak di frame untuk dialihkan kunciannya.
                        box_motion = compute_box_motion(frame_diff, target_box) if frame_diff is not None else 2.0
                        if box_motion < 1.3 and abs(smooth_err_x) < 22:
                            static_streak += 1
                        else:
                            static_streak = max(0, static_streak - 2)

                        # Jika terkunci pada benda mati selama >= 10 frame (~0.33 detik):
                        if static_streak >= 10:
                            cand_box, cand_score = find_target_in_frame(frame, target_clothes_hist, box_w, box_h,
                                                                        yunet_detector=yunet_detector,
                                                                        face_cascade=face_cascade,
                                                                        profile_type=current_profile,
                                                                        frame_diff=frame_diff)
                            if cand_box and cand_score >= 0.30:
                                cand_mot = compute_box_motion(frame_diff, cand_box) if frame_diff is not None else 0.0
                                if cand_mot > 1.8 or cand_score > match_score + 0.08:
                                    print(f"[ANTI-BENDA-MATI] Target diam (mot={box_motion:.1f}), beralih ke majikan bergerak di {cand_box} ({int(cand_score*100)}%, mot={cand_mot:.1f})!")
                                    tracker = create_tracker(current_mode)
                                    if tracker:
                                        tracker.init(frame, cand_box)
                                        target_box = list(cand_box)
                                        static_streak = 0
                                        mismatch_streak = 0
                                        match_score = cand_score
                                        clothes_pct = int(match_score * 100)
                            elif static_streak >= 25 and match_score < 0.38:
                                # Sudah diam >= 25 frame dan skor baju rendah -> lepas kuncian benda mati
                                print(f"[ANTI-BENDA-MATI] Target terkonfirmasi benda mati tak bergerak (mot={box_motion:.1f}, match={clothes_pct}%). Melepas kuncian!")
                                tracking_active = False
                                target_box = None
                                static_streak = 0
                                scanning_owner_start = now
                                status_text = "SCANNING_OWNER"
                                status_color = (0, 165, 255)

                        if target_box is not None:
                            is_valid_match = (match_score >= 0.32) or (ai_reanchored and match_score >= 0.20)
                            if is_valid_match:
                                mismatch_streak = 0
                                status_text = f"LOCKED_TRACKING ({clothes_pct}%)"
                                status_color = (0, 255, 0)
                            else:
                                mismatch_streak += 1
                                if mismatch_streak < 20:
                                    # Masih dalam batas toleransi sesaat (bayangan, putaran badan, lengan lewat)
                                    status_text = f"TRACKING_SOFT ({clothes_pct}%)"
                                    status_color = (0, 200, 255)
                                else:
                                    status_text = f"COLOR_MISMATCH ({clothes_pct}%)"
                                    status_color = (0, 165, 255)

                                    # Jika benar-benar mismatch >= 20 frame berturut-turut (~0.7s):
                                    re_box, re_score = find_target_in_frame(frame, target_clothes_hist, box_w, box_h,
                                                                           yunet_detector=yunet_detector,
                                                                           face_cascade=face_cascade,
                                                                           profile_type=current_profile,
                                                                           last_target_box=target_box,
                                                                           frame_diff=frame_diff)
                                    if re_box and re_score >= 0.30:
                                        print(f"[RE-SNAP] Melepas objek salah, memaksa kotaki pemilik di {re_box} ({int(re_score*100)}%)!")
                                        tracker = create_tracker(current_mode)
                                        tracker.init(frame, re_box)
                                        target_box = list(re_box)
                                        mismatch_streak = 0
                                        static_streak = 0
                                        status_text = f"LOCKED_TRACKING ({int(re_score * 100)}%)"
                                        status_color = (0, 255, 0)
                                    else:
                                        tracking_active = False
                                        target_box = None
                                        static_streak = 0
                                        scanning_owner_start = now
                                        status_text = "SCANNING_OWNER"
                                        status_color = (0, 165, 255)
                    else:
                        # Tracker lepas (misal gerakan sangat cepat / ngereog):
                        re_box, re_score = find_target_in_frame(frame, target_clothes_hist, box_w, box_h,
                                                               yunet_detector=yunet_detector,
                                                               face_cascade=face_cascade,
                                                               profile_type=current_profile,
                                                               last_target_box=target_box,
                                                               frame_diff=frame_diff)
                        if re_box and re_score >= 0.30:
                            print(f"[RE-SNAP] Target pemilik ditemukan ({int(re_score*100)}%)! Langsung mengotaki...")
                            tracker = create_tracker(current_mode)
                            tracker.init(frame, re_box)
                            tracking_active = True
                            target_box = list(re_box)
                            mismatch_streak = 0
                            static_streak = 0
                            scanning_owner_start = None
                            status_text = f"LOCKED_TRACKING ({int(re_score * 100)}%)"
                            status_color = (0, 255, 0)
                        else:
                            tracking_active = False
                            target_box = None
                            mismatch_streak = 0
                            static_streak = 0
                            if scanning_owner_start is None:
                                scanning_owner_start = now
                            status_text = "SCANNING_OWNER"
                            status_color = (0, 0, 255)

            # --- METODE 2: FACE DETECTOR (AI YUNET + CASCADE FALLBACK) ---
            elif current_mode == "face":
                ai_faces = run_yunet_detection(yunet_detector, frame) if yunet_detector else []
                if len(ai_faces) > 0:
                    fx, fy, fw, fh, _ = ai_faces[0]
                    target_box = (fx, fy, fw, fh)
                    status_text = "AI_FACE_DETECTED"
                    status_color = (0, 255, 0)
                elif face_cascade and not face_cascade.empty():
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=4, minSize=(22, 22))
                    if len(faces) > 0:
                        faces = sorted(faces, key=lambda b: b[2] * b[3], reverse=True)
                        target_box = faces[0]
                        status_text = "FACE_DETECTED"
                        status_color = (0, 255, 0)
                    else:
                        status_text = "NO_FACE"
                        status_color = (0, 0, 255)
                else:
                    status_text = "NO_DETECTOR"
                    status_color = (0, 0, 255)

            # Hitung Deviasi Piksel & Estimasi Jarak Monokular
            err_x = 0
            err_y = 0
            has_target = (target_box is not None) and (mismatch_streak < 20)

            if has_target:
                tx, ty, tw, th = target_box
                cx = tx + tw // 2
                cy = ty + th // 2
                err_x = cx - CENTER_X
                err_y = cy - CENTER_Y

                prof_data = TARGET_PROFILES.get(current_profile, TARGET_PROFILES["torso"])

                # Hitung Jarak Monokular Aktual
                if current_mode == "face":
                    instant_dist = (0.20 * FOCAL_LENGTH_PX) / max(8, th)
                else:
                    face_calibrated = False
                    if yunet_detector and (frame_counter % 5 == 0):
                        ai_faces = run_yunet_detection(yunet_detector, frame)
                        for (fx, fy, fw, fh, conf) in ai_faces:
                            if abs((fx + fw // 2) - cx) < tw * 0.85:
                                instant_dist = (0.20 * FOCAL_LENGTH_PX) / max(8, fh)
                                face_calibrated = True
                                break

                    if not face_calibrated and face_cascade and (frame_counter % 8 == 0):
                        roi_y1 = max(0, ty - int(th * 0.45))
                        roi_y2 = min(FRAME_HEIGHT, ty + int(th * 0.35))
                        roi_x1 = max(0, tx - 10)
                        roi_x2 = min(FRAME_WIDTH, tx + tw + 10)
                        if (roi_y2 - roi_y1 >= 25) and (roi_x2 - roi_x1 >= 25):
                            roi_gray = cv2.cvtColor(frame[roi_y1:roi_y2, roi_x1:roi_x2], cv2.COLOR_BGR2GRAY)
                            sub_faces = face_cascade.detectMultiScale(roi_gray, scaleFactor=1.2, minNeighbors=3, minSize=(20, 20))
                            if len(sub_faces) > 0:
                                sub_faces = sorted(sub_faces, key=lambda b: b[2] * b[3], reverse=True)
                                _, _, _, sfh = sub_faces[0]
                                instant_dist = (0.20 * FOCAL_LENGTH_PX) / max(8, sfh)
                                face_calibrated = True

                    if not face_calibrated:
                        instant_dist = (prof_data["real_h"] * FOCAL_LENGTH_PX) / max(10, th)

                instant_dist = max(0.4, min(6.0, instant_dist))
                if smooth_distance <= 0.0:
                    smooth_distance = instant_dist
                else:
                    smooth_distance = 0.25 * instant_dist + 0.75 * smooth_distance

                if smooth_distance < 0.95:
                    dist_status = "TERLALU DEKAT (<1.0m)"
                    dist_badge_color = (0, 0, 255)
                elif smooth_distance <= 2.20:
                    dist_status = "IDEAL FOLLOW (1.0-2.2m)"
                    dist_badge_color = (0, 255, 0)
                else:
                    dist_status = "JAUH (>2.2m)"
                    dist_badge_color = (0, 255, 255)

                # Gambar Bounding Box Target & Garis Vektor Deviasi
                cv2.rectangle(annotated_frame, (tx, ty), (tx + tw, ty + th), status_color, 2)
                cv2.circle(annotated_frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.line(annotated_frame, (CENTER_X, CENTER_Y), (cx, cy), (255, 255, 0), 2)

                # Badge label target & jarak di atas kotak
                tag_label = f"{prof_data['label']} | {smooth_distance:.2f}m"
                if current_mode in ["kcf", "mosse"]:
                    tag_label += f" | Baju:{vision_state.clothes_match_pct}%"
                cv2.rectangle(annotated_frame, (tx, max(0, ty - 18)), (tx + len(tag_label) * 8 + 6, max(18, ty)), (0, 0, 0), -1)
                cv2.putText(annotated_frame, tag_label, (tx + 3, max(13, ty - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, dist_badge_color, 1)
            else:
                smooth_distance = 0.0
                dist_status = "MENCARI TARGET..."

            # Gambar Titik Pusat Layar (Center Target Crosshair)
            cv2.drawMarker(annotated_frame, (CENTER_X, CENTER_Y), (200, 200, 200),
                           markerType=cv2.MARKER_CROSS, markerSize=16, thickness=1)
            cv2.rectangle(annotated_frame, (CENTER_X - 12, CENTER_Y - 12), (CENTER_X + 12, CENTER_Y + 12),
                          (100, 100, 100), 1)

            # Transmisi Serial Asinkronus ke ESP32 (0.00 ms, tidak pernah memblokir FPS)
            now_time = time.time()
            if has_target:
                # 1. Update AI Online Learning (Self-training anti-overshoot & anti-jitter)
                with vision_state.lock:
                    is_autotune = vision_state.auto_tune
                    cur_speed = vision_state.servo_speed_pct
                    cur_smooth = vision_state.servo_smoothing_pct
                    deadzone = vision_state.servo_deadzone

                if is_autotune:
                    new_spd, new_sm, new_dz, tune_status, metrics_str = servo_learner.step(err_x, cur_speed, cur_smooth, True)
                    if (new_spd != cur_speed or new_sm != cur_smooth or new_dz != deadzone
                            or tune_status != vision_state.auto_tune_status or metrics_str != vision_state.auto_tune_metrics):
                        with vision_state.lock:
                            vision_state.servo_speed_pct = new_spd
                            vision_state.servo_smoothing_pct = new_sm
                            vision_state.servo_deadzone = new_dz
                            vision_state.auto_tune_status = tune_status
                            vision_state.auto_tune_metrics = metrics_str
                        cur_speed, cur_smooth, deadzone = new_spd, new_sm, new_dz

                # 2. Filter Exponential Moving Average (EMA) - Mengubah sentakan kasar jadi luncuran halus
                alpha = max(0.10, min(0.60, 1.0 - (cur_smooth / 100.0) * 0.85))
                smooth_err_x = alpha * err_x + (1.0 - alpha) * smooth_err_x
                smooth_err_y = alpha * err_y + (1.0 - alpha) * smooth_err_y

                # 3. Dynamic Deadzone: Mencegah servo bergetar saat target sudah dekat di tengah
                dead_x = smooth_err_x if abs(smooth_err_x) > deadzone else 0.0
                dead_y = smooth_err_y if abs(smooth_err_y) > deadzone else 0.0

                # 4. Pengali Kecepatan Respon (cur_speed: 10% - 100% -> scale: 0.15 - 1.0)
                speed_factor = max(0.15, min(1.0, cur_speed / 100.0))
                target_send_x = int(dead_x * speed_factor)
                target_send_y = int(dead_y * speed_factor)

                dist_cm = int(smooth_distance * 100)
                send_err_x = -target_send_x if do_invert_pan else target_send_x
                send_err_y = -target_send_y if do_invert_tilt else target_send_y
                serial_sender.send(f"X:{send_err_x},Y:{send_err_y},D:{dist_cm}\n")
            else:
                smooth_err_x = 0.0
                smooth_err_y = 0.0
                with vision_state.lock:
                    vision_state.auto_tune_status = "MENUNGGU TARGET"
                    vision_state.auto_tune_metrics = servo_learner.metrics_str
                serial_sender.send("LOST\n")
            serial_state_str = serial_sender.state_str

            # Hitung FPS
            fps = 1.0 / (now_time - prev_time + 1e-6)
            prev_time = now_time

            # Gambar Overlay Teks HUD
            mirror_tag = " [MIRROR]" if do_mirror else ""
            if do_flip_v:
                mirror_tag += " [FLIP-V]"
            if do_invert_pan:
                mirror_tag += " [INV-PAN]"
            cv2.putText(annotated_frame, f"FPS: {fps:.1f} | {current_mode.upper()}{mirror_tag}", (10, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(annotated_frame, f"Status: {status_text}", (10, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_color, 1)
            if has_target:
                cv2.putText(annotated_frame, f"ErrX: {err_x:+d}px | ErrY: {err_y:+d}px | Jarak: {smooth_distance:.2f}m", (10, FRAME_HEIGHT - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
            elif mismatch_streak >= 3:
                cv2.putText(annotated_frame, f"FAILSAFE: OBJEK LAIN DITOLAK ({vision_state.clothes_match_pct}%)", (10, FRAME_HEIGHT - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 165, 255), 1)

            # Update State untuk Web Dashboard
            with vision_state.lock:
                vision_state.mode = current_mode
                vision_state.target_profile = current_profile
                vision_state.status = status_text
                vision_state.fps = fps
                vision_state.err_x = err_x
                vision_state.err_y = err_y
                vision_state.has_target = has_target
                vision_state.distance_m = smooth_distance if has_target else 0.0
                vision_state.distance_status = dist_status if has_target else "SEARCHING"
                vision_state.serial_state = serial_state_str

            # Encode JPEG jika ada client Web HUD aktif, atau berkala (1 detik) untuk snapshot/preview
            should_encode = False
            with vision_state.lock:
                active_users = vision_state.active_clients

            if not args.no_web:
                if active_users > 0:
                    should_encode = True
                elif now_time - last_idle_encode > 1.0:
                    should_encode = True
                    last_idle_encode = now_time

            if should_encode:
                ret_enc, jpeg = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret_enc:
                    with vision_state.lock:
                        vision_state.latest_jpeg = jpeg.tobytes()

            # Output Telemetri ke Terminal SSH (Headless)
            if is_headless:
                if now_time - last_terminal_print > 0.25:
                    if has_target:
                        clothes_tag = f" | Baju: {vision_state.clothes_match_pct}%" if current_mode in ["kcf", "mosse"] else ""
                        print(f"[{status_text:^20}] FPS: {fps:4.1f} | ErrX: {err_x:+4d} px | ErrY: {err_y:+4d} px | Jarak: {smooth_distance:4.2f} m{clothes_tag} | Serial: {serial_state_str}")
                    else:
                        print(f"[{status_text:^20}] FPS: {fps:4.1f} | Mencari Target...         | Serial: {serial_state_str}")
                    last_terminal_print = now_time
                time.sleep(0.005)
            else:
                # Tampilkan di HDMI monitor jika terpasang layar
                cv2.imshow("STB Vision Tracker", annotated_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    tracking_active = False
                    target_clothes_hist = None
                    countdown_start = None
                    active_lock_box = None
                    human_last_seen = 0.0
                elif key == ord('m'):
                    current_mode = "face" if current_mode == "kcf" else "kcf"
                    tracking_active = False
                    target_clothes_hist = None
                    countdown_start = None
                    active_lock_box = None
                    human_last_seen = 0.0
                elif key == ord('f'):
                    with vision_state.lock:
                        vision_state.mirror = not vision_state.mirror
                    tracking_active = False
                    target_clothes_hist = None
                    countdown_start = None
                    active_lock_box = None
                    human_last_seen = 0.0
                elif key == ord('v'):
                    with vision_state.lock:
                        vision_state.flip_v = not vision_state.flip_v
                    tracking_active = False
                    target_clothes_hist = None
                    countdown_start = None
                    active_lock_box = None
                    human_last_seen = 0.0
                elif key == ord('p'):
                    profiles = ["body", "torso", "face"]
                    idx = (profiles.index(current_profile) + 1) % len(profiles)
                    current_profile = profiles[idx]
                    with vision_state.lock:
                        vision_state.target_profile = current_profile
                    tracking_active = False
                    target_clothes_hist = None
                    countdown_start = None
                    active_lock_box = None
                    human_last_seen = 0.0

    except KeyboardInterrupt:
        print("\n[STOP] Program dihentikan pengguna.")
    finally:
        if serial_sender:
            serial_sender.close()
        cap.release()
        if not is_headless:
            cv2.destroyAllWindows()
        print("[SHUTDOWN] Kamera & serial ditutup. Selesai.")

if __name__ == "__main__":
    main()
