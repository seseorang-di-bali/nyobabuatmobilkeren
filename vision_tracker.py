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
import cv2
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
        "box": (115, 155),
        "real_h": 0.80,
        "label": "BAJU & PUNGGUNG"
    },
    "body": {
        "name": "Badan Penuh & Celana (Full Body)",
        "box": (120, 200),
        "real_h": 1.65,
        "label": "BADAN & CELANA"
    },
    "face": {
        "name": "Wajah Saja (Face)",
        "box": (85, 105),
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
        cw = max(32, min(FRAME_WIDTH, int(fw * 2.5)))
        ch = max(42, min(FRAME_HEIGHT, int(fh * 3.3)))
        top_y = fy + int(fh * 0.70)
    elif profile_type == "body":
        cw = max(40, min(FRAME_WIDTH, int(fw * 2.8)))
        ch = max(70, min(FRAME_HEIGHT, int(fh * 6.5)))
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
                return None, None
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            # 2D HS: 16 bin Hue, 12 bin Saturation (warna kain murni)
            hist_hs = cv2.calcHist([hsv], [0, 1], None, [16, 12], [0, 180, 0, 256])
            cv2.normalize(hist_hs, hist_hs, alpha=1.0, norm_type=cv2.NORM_L1)
            # 1D V: 8 bin Value (tingkat gelap-terang)
            hist_v = cv2.calcHist([hsv], [2], None, [8], [0, 256])
            cv2.normalize(hist_v, hist_v, alpha=1.0, norm_type=cv2.NORM_L1)
            return hist_hs, hist_v

        u_hs, u_v = compute_patch_hist(roi_upper)
        l_hs, l_v = compute_patch_hist(roi_lower)

        if u_hs is None and l_hs is None:
            return None

        return {
            "u_hs": u_hs,
            "u_v": u_v,
            "l_hs": l_hs,
            "l_v": l_v
        }
    except Exception:
        return None

def compare_clothes_color(frame, box, target_sig):
    """
    Memeriksa kecocokan pakaian secara Dual-Zone (Baju Atas + Celana):
    - 75% bobot pada Hue-Saturation (warna kain asli).
    - 25% bobot pada Value (kecerahan).
    - Skor gabungan: 60% Baju Atas + 40% Celana/Bawah.
    Jika orang lain lewat memakai baju mirip tetapi celananya berbeda, target langsung ditolak!
    """
    if target_sig is None:
        return 1.0
    curr_sig = get_clothes_color_signature(frame, box)
    if curr_sig is None:
        return 0.5
    try:
        # Dukungan jika format lama (single numpy array)
        if not isinstance(target_sig, dict):
            return 0.7

        def compare_patch(t_hs, t_v, c_hs, c_v):
            if t_hs is None or c_hs is None:
                return 0.5
            s_hs = float(cv2.compareHist(t_hs, c_hs, cv2.HISTCMP_INTERSECT))
            s_v = float(cv2.compareHist(t_v, c_v, cv2.HISTCMP_INTERSECT)) if (t_v is not None and c_v is not None) else 0.5
            return 0.75 * s_hs + 0.25 * s_v

        score_upper = compare_patch(target_sig.get("u_hs"), target_sig.get("u_v"),
                                    curr_sig.get("u_hs"), curr_sig.get("u_v"))
        score_lower = compare_patch(target_sig.get("l_hs"), target_sig.get("l_v"),
                                    curr_sig.get("l_hs"), curr_sig.get("l_v"))

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
        return updated
    except Exception:
        return target_sig

def find_target_in_frame(frame, target_hist, base_box_w, base_box_h, yunet_detector=None, face_cascade=None, profile_type="torso", last_target_box=None):
    """
    Sistem Persistent Re-Identification (Re-ID) & Smart Clothes Matcher:
    1. Prioritas 1: YuNet AI Neural Network Detector (~5-9 ms).
    2. Prioritas 2: Haar Cascade Fallback.
    3. Prioritas 3: Full-Frame Multi-Scale Grid Scan untuk tampak punggung.
    Menerapkan Spatial Continuity Gating (anti-teleport) agar tidak melompat ke orang asing.
    """
    if target_hist is None:
        return None, 0.0

    best_box = None
    best_score = 0.0

    def calculate_effective_score(cand_b, raw_s):
        if last_target_box is None:
            return raw_s
        lx = last_target_box[0] + last_target_box[2] // 2
        ly = last_target_box[1] + last_target_box[3] // 2
        cx = cand_b[0] + cand_b[2] // 2
        cy = cand_b[1] + cand_b[3] // 2
        dist = ((cx - lx) ** 2 + (cy - ly) ** 2) ** 0.5
        prox_factor = max(0.70, 1.0 - (dist / 380.0))
        return raw_s * prox_factor

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

            if best_score >= 0.36:
                return best_box, max(0.55, best_score)

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

                if best_score >= 0.42:
                    return best_box, best_score
        except Exception:
            pass

    # 3. Prioritas Ketiga: Full-Frame Multi-Scale Grid Scan (Mencakup seluruh area vertikal frame untuk tampak punggung)
    scales = [0.55, 0.75, 1.0, 1.25]
    for s in scales:
        cw = max(32, min(FRAME_WIDTH - 10, int(base_box_w * s)))
        ch = max(42, min(FRAME_HEIGHT - 10, int(base_box_h * s)))
        step_x = max(16, int(cw * 0.22))
        step_y = max(20, int(ch * 0.25))

        for x_cand in range(0, max(1, FRAME_WIDTH - cw + 1), step_x):
            for y_cand in range(10, max(11, FRAME_HEIGHT - ch + 1), step_y):
                cand_box = (x_cand, y_cand, cw, ch)
                score = compare_clothes_color(frame, cand_box, target_hist)
                eff_s = calculate_effective_score(cand_box, score)
                if eff_s > best_score:
                    best_score = eff_s
                    best_box = cand_box

    if best_score >= 0.52:
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
            print(f"[CAMERA] Optimasi hardware v4l2 diterapkan pada {cam_device} (Backlight Comp=1, Anti-Flicker=50Hz).")
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
        self.mirror = True       # Default mirror horizontal ON
        self.flip_v = False      # Flip vertikal (jika kamera terbalik atas-bawah)
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
                <button id="btn-toggle-mirror" class="btn-primary" onclick="toggleMirror()">🪞 Mirror (Kiri/Kanan)</button>
                <button id="btn-toggle-flip-v" class="btn-secondary" onclick="toggleFlipV()">↕️ Flip Vertikal</button>
            </div>
        </div>

        <footer>
            Autonomous Vision-Guided Robot &bull; Streamed from STB Armbian to Mac Local Browser
        </footer>
    </div>

    <script>
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

                    // Highlight tombol toggle mirror & flip-v & anti-silau
                    const btnMirror = document.getElementById('btn-toggle-mirror');
                    if (btnMirror) {
                        btnMirror.className = data.mirror ? 'btn-primary' : 'btn-secondary';
                    }
                    const btnFlipV = document.getElementById('btn-toggle-flip-v');
                    if (btnFlipV) {
                        btnFlipV.className = data.flip_v ? 'btn-primary' : 'btn-secondary';
                    }
                    const btnWdr = document.getElementById('btn-toggle-wdr');
                    if (btnWdr) {
                        btnWdr.className = data.anti_silau ? 'btn-primary' : 'btn-secondary';
                        btnWdr.innerText = data.anti_silau ? '☀️ Anti-Silau (WDR ON)' : '☀️ Anti-Silau (WDR OFF)';
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
    parser.add_argument("--no-mirror", action="store_true", help="Nonaktifkan mirror horizontal (kamera selfie)")
    parser.add_argument("--flip-v", action="store_true", help="Aktifkan flip vertikal (jika kamera terbalik atas-bawah)")
    parser.add_argument("--no-wdr", action="store_true", help="Nonaktifkan filter Anti-Silau (Wide Dynamic Range CLAHE)")
    args = parser.parse_args()

    vision_state.mirror = not args.no_mirror
    vision_state.flip_v = args.flip_v
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
    print(f" - Mirror   : {'ON (Horizontal)' if vision_state.mirror else 'OFF'}{' + [FLIP-V]' if vision_state.flip_v else ''}")
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

    # Inisialisasi Serial ke ESP32
    ser = None
    serial_state_str = "DRY"
    if not args.dry_run:
        port_to_use = args.port or find_serial_port()
        if port_to_use:
            try:
                ser = serial.Serial(port_to_use, 115200, timeout=0.05)
                time.sleep(2.0)
                serial_state_str = "TX_OK"
                print(f"[SERIAL] Sukses terhubung ke ESP32 pada: {port_to_use}")
            except Exception as e:
                print(f"[SERIAL WARN] Gagal membuka port {port_to_use}: {e}")
                print("[SERIAL WARN] Tips: jalankan 'sudo usermod -a -G dialout $USER'")
        else:
            print("[SERIAL INFO] ESP32 tidak ditemukan. Berjalan dalam mode DRY-RUN.")
    else:
        print("[SERIAL INFO] Mode simulasi DRY-RUN aktif.")

    vision_state.serial_state = serial_state_str
    vision_state.serial_port = port_to_use if (ser and not args.dry_run) else "DRY_RUN"
    vision_state.mode = args.mode

    # Inisialisasi Kamera UVC
    cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2 if sys.platform.startswith('linux') else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print(f"[ERROR] Tidak dapat membuka kamera pada index {cam_index} ({cam_device})!")
        if sys.platform == 'darwin':
            print("[MACOS TIPS] Izin kamera belum diberikan ke aplikasi Terminal / iTerm.")
            print("[MACOS TIPS] Buka: System Settings -> Privacy & Security -> Camera, lalu pastikan Terminal / iTerm diizinkan (ON).")
        else:
            print("[ERROR] Cek apakah webcam USB sudah terhubung dengan: ls -l /dev/video*")
        return

    # Optimasi Hardware UVC Camera (Backlight Compensation, Anti-Silau & Anti-Flicker)
    configure_camera_hardware(cap, cam_device)

    # Inisialisasi Detektor Wajah (YuNet Neural Network + Haar Cascade Fallback)
    face_cascade = get_cascade_classifier()
    yunet_detector = get_ai_face_detector()

    # Parameter Tracking Bounding Box & Target Profile
    current_profile = args.profile
    prof_info = TARGET_PROFILES.get(current_profile, TARGET_PROFILES["torso"])
    box_w, box_h = prof_info["box"]
    init_box = ((FRAME_WIDTH - box_w) // 2, (FRAME_HEIGHT - box_h) // 2, box_w, box_h)
    active_lock_box = init_box
    tracker = None
    target_clothes_hist = None
    tracking_active = False
    lock_countdown_start = time.time()
    countdown_duration = 3.0
    smooth_distance = 0.0
    dist_status = "SEARCHING"
    last_face_scan = 0
    frame_counter = 0
    mismatch_streak = 0

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
            if not ret:
                print("[WARN] Gagal membaca frame dari webcam.")
                time.sleep(0.05)
                continue

            frame_counter += 1
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            with vision_state.lock:
                do_mirror = vision_state.mirror
                do_flip_v = vision_state.flip_v
                do_anti_silau = vision_state.anti_silau

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
                    countdown_duration = 3.0
                    vision_state.clothes_match_pct = 100
                    lock_countdown_start = time.time()
                    vision_state.trigger_reset = False
                    print("[CONTROL] Reset pelacakan dari Web HUD diterima.")

                if vision_state.change_profile_req:
                    current_profile = vision_state.change_profile_req
                    vision_state.target_profile = current_profile
                    vision_state.change_profile_req = None
                    prof_info = TARGET_PROFILES.get(current_profile, TARGET_PROFILES["torso"])
                    box_w, box_h = prof_info["box"]
                    init_box = ((FRAME_WIDTH - box_w) // 2, (FRAME_HEIGHT - box_h) // 2, box_w, box_h)
                    active_lock_box = init_box
                    tracking_active = False
                    target_clothes_hist = None
                    mismatch_streak = 0
                    countdown_duration = 3.0
                    vision_state.clothes_match_pct = 100
                    lock_countdown_start = time.time()
                    print(f"[CONTROL] Profil target diubah ke '{prof_info['name']}'.")

                if vision_state.change_mode_req:
                    current_mode = vision_state.change_mode_req
                    vision_state.mode = current_mode
                    vision_state.change_mode_req = None
                    tracking_active = False
                    target_clothes_hist = None
                    mismatch_streak = 0
                    countdown_duration = 3.0
                    vision_state.clothes_match_pct = 100
                    lock_countdown_start = time.time()
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
                    # Jangan dipaksa hitung mundur/ngulang, langsung cari pemilik di frame secara multi-skala!
                    if target_clothes_hist is not None:
                        re_box, re_score = find_target_in_frame(frame, target_clothes_hist, box_w, box_h,
                                                               yunet_detector=yunet_detector,
                                                               face_cascade=face_cascade,
                                                               profile_type=current_profile)
                        if re_box and re_score >= 0.52:
                            tracker = create_tracker(current_mode)
                            if tracker:
                                tracker.init(frame, re_box)
                                tracking_active = True
                                target_box = list(re_box)
                                mismatch_streak = 0
                                initial_th = re_box[3]
                                smooth_distance = (prof_info["real_h"] * FOCAL_LENGTH_PX) / max(10, initial_th)
                                status_text = f"LOCKED_TRACKING ({int(re_score * 100)}%)"
                                status_color = (0, 255, 0)
                                print(f"[RE-SNAP] Pemilik terdeteksi ({int(re_score * 100)}% match)! Memaksa mengotaki target...")
                        else:
                            status_text = "SCANNING_OWNER"
                            status_color = (0, 165, 255)
                            cx_box = ((FRAME_WIDTH - box_w) // 2, (FRAME_HEIGHT - box_h) // 2, box_w, box_h)
                            cv2.rectangle(annotated_frame, (cx_box[0], cx_box[1]),
                                          (cx_box[0] + cx_box[2], cx_box[1] + cx_box[3]), (0, 165, 255), 1)
                            cv2.putText(annotated_frame, "MENCARI PEMILIK (MEMORI TERSIMPAN)...", (10, FRAME_HEIGHT - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 165, 255), 1)

                    # KASUS B: BELUM ADA TEMPLATE DI MEMORI (Awal Start atau User Klik Reset)
                    else:
                        center_box = ((FRAME_WIDTH - box_w) // 2, (FRAME_HEIGHT - box_h) // 2, box_w, box_h)

                        # Smart Human Detection: Cek keberadaan manusia sebelum memulai hitung mundur
                        ai_detected = False
                        if now - last_face_scan > 0.08:
                            last_face_scan = now
                            if yunet_detector:
                                ai_faces = run_yunet_detection(yunet_detector, frame)
                                if len(ai_faces) > 0:
                                    fx, fy, fw, fh, _ = ai_faces[0]
                                    active_lock_box = derive_body_box_from_face(fx, fy, fw, fh, current_profile)
                                    ai_detected = True

                            if not ai_detected and face_cascade:
                                try:
                                    gray_snap = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                                    faces_found = face_cascade.detectMultiScale(gray_snap, scaleFactor=1.2, minNeighbors=3, minSize=(20, 20))
                                    if len(faces_found) > 0:
                                        faces_found = sorted(faces_found, key=lambda b: b[2] * b[3], reverse=True)
                                        fx, fy, fw, fh = faces_found[0]
                                        active_lock_box = derive_body_box_from_face(fx, fy, fw, fh, current_profile)
                                        ai_detected = True
                                except Exception:
                                    pass

                        if not ai_detected:
                            # TIDAK ADA MANUSIA: Jangan pernah hitung mundur dan jangan mengunci pintu/tembok kosong!
                            lock_countdown_start = now  # Reset timer terus-menerus
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
                            # ADA MANUSIA TERDETEKSI: Jalankan hitung mundur 3 detik untuk merekam target
                            elapsed = now - lock_countdown_start
                            remaining = countdown_duration - elapsed
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
                                    with vision_state.lock:
                                        vision_state.clothes_match_pct = 100
                                    initial_th = active_lock_box[3]
                                    smooth_distance = (prof_info["real_h"] * FOCAL_LENGTH_PX) / max(10, initial_th)
                                    print(f"[TRACKER] Target manusia terkunci & disimpan di memori ({prof_info['name']}) via {current_mode.upper()}!")
                                else:
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
                        # Jika kecocokan sangat tinggi (>= 68%), adaptasikan sedikit variasi pencahayaan
                        if match_score >= 0.68:
                            curr_hist = get_clothes_color_signature(frame, target_box)
                            if curr_hist is not None and target_clothes_hist is not None:
                                target_clothes_hist = update_clothes_color_signature(target_clothes_hist, curr_hist, alpha=0.96)

                        is_valid_match = (match_score >= 0.55) or (ai_reanchored and match_score >= 0.36)
                        if is_valid_match:
                            mismatch_streak = 0
                            status_text = f"LOCKED_TRACKING ({clothes_pct}%)"
                            status_color = (0, 255, 0)
                        else:
                            mismatch_streak += 1
                            status_text = f"COLOR_MISMATCH ({clothes_pct}%)"
                            status_color = (0, 165, 255)

                            # Jika menempel ke objek salah (seperti pintu/tembok selama >= 3 frame):
                            # Langsung cari pemilik di seluruh frame secara multi-skala dan paksa kotaki kembali!
                            if mismatch_streak >= 3:
                                re_box, re_score = find_target_in_frame(frame, target_clothes_hist, box_w, box_h,
                                                                       yunet_detector=yunet_detector,
                                                                       face_cascade=face_cascade,
                                                                       profile_type=current_profile,
                                                                       last_target_box=target_box)
                                if re_box and re_score >= 0.52:
                                    print(f"[RE-SNAP] Melepas objek salah, memaksa kotaki pemilik di {re_box} ({int(re_score*100)}%)!")
                                    tracker = create_tracker(current_mode)
                                    tracker.init(frame, re_box)
                                    target_box = list(re_box)
                                    mismatch_streak = 0
                                    status_text = f"LOCKED_TRACKING ({int(re_score * 100)}%)"
                                    status_color = (0, 255, 0)
                                else:
                                    tracking_active = False
                                    target_box = None
                                    status_text = "SCANNING_OWNER"
                                    status_color = (0, 165, 255)
                    else:
                        # Tracker lepas (misal gerakan cepat / ngereog):
                        re_box, re_score = find_target_in_frame(frame, target_clothes_hist, box_w, box_h,
                                                               yunet_detector=yunet_detector,
                                                               face_cascade=face_cascade,
                                                               profile_type=current_profile,
                                                               last_target_box=target_box)
                        if re_box and re_score >= 0.52:
                            print(f"[RE-SNAP] Target pemilik ditemukan ({int(re_score*100)}%)! Langsung mengotaki...")
                            tracker = create_tracker(current_mode)
                            tracker.init(frame, re_box)
                            tracking_active = True
                            target_box = list(re_box)
                            mismatch_streak = 0
                            status_text = f"LOCKED_TRACKING ({int(re_score * 100)}%)"
                            status_color = (0, 255, 0)
                        else:
                            tracking_active = False
                            target_box = None
                            mismatch_streak = 0
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
            has_target = (target_box is not None) and (mismatch_streak < 2)

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

            # Transmisi Serial ke ESP32 (~30 FPS)
            now_time = time.time()
            if ser and (now_time - last_serial_send > 0.03):
                try:
                    if has_target:
                        dist_cm = int(smooth_distance * 100)
                        msg = f"X:{err_x},Y:{err_y},D:{dist_cm}\n"
                    else:
                        msg = "LOST\n"
                    ser.write(msg.encode('utf-8'))
                    last_serial_send = now_time
                except Exception as e:
                    print(f"[SERIAL ERROR] Komunikasi terputus: {e}")
                    ser = None
                    serial_state_str = "DISCONNECTED"
                    vision_state.serial_state = "DISCONNECTED"
            elif ser is None and not args.dry_run and (now_time - last_serial_reconnect > 2.0):
                # Percobaan Auto-Reconnect berkala jika kabel ESP32 dicabut-pasang
                last_serial_reconnect = now_time
                reconnect_port = args.port or find_serial_port()
                if reconnect_port:
                    try:
                        ser = serial.Serial(reconnect_port, 115200, timeout=0.05)
                        serial_state_str = "TX_OK"
                        vision_state.serial_state = "TX_OK"
                        vision_state.serial_port = reconnect_port
                        print(f"[SERIAL] Auto-reconnect berhasil ke ESP32 pada: {reconnect_port}")
                    except Exception:
                        ser = None

            # Hitung FPS
            fps = 1.0 / (now_time - prev_time + 1e-6)
            prev_time = now_time

            # Gambar Overlay Teks HUD
            mirror_tag = " [MIRROR]" if do_mirror else ""
            if do_flip_v:
                mirror_tag += " [FLIP-V]"
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
                    lock_countdown_start = time.time()
                elif key == ord('m'):
                    current_mode = "face" if current_mode == "kcf" else "kcf"
                    tracking_active = False
                    lock_countdown_start = time.time()
                elif key == ord('f'):
                    with vision_state.lock:
                        vision_state.mirror = not vision_state.mirror
                    tracking_active = False
                    lock_countdown_start = time.time()
                elif key == ord('v'):
                    with vision_state.lock:
                        vision_state.flip_v = not vision_state.flip_v
                    tracking_active = False
                    lock_countdown_start = time.time()
                elif key == ord('p'):
                    profiles = ["body", "torso", "face"]
                    idx = (profiles.index(current_profile) + 1) % len(profiles)
                    current_profile = profiles[idx]
                    with vision_state.lock:
                        vision_state.target_profile = current_profile
                    tracking_active = False
                    lock_countdown_start = time.time()

    except KeyboardInterrupt:
        print("\n[STOP] Program dihentikan pengguna.")
    finally:
        if ser:
            try:
                ser.write(b"LOST\n")
                ser.close()
            except Exception:
                pass
        cap.release()
        if not is_headless:
            cv2.destroyAllWindows()
        print("[SHUTDOWN] Kamera & serial ditutup. Selesai.")

if __name__ == "__main__":
    main()
