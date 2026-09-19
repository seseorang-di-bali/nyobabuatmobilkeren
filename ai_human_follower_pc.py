#!/usr/bin/env python3
"""
ai_human_follower_pc.py - Xiaomi / DJI Style High-Precision Full-Body AI Tracker
================================================================================
Dirancang khusus untuk PC (Pop!_OS / Ubuntu / Linux / Windows) dengan Ryzen 5 / Intel.

Fitur Unggulan:
1. Menggunakan YOLOv8 Neural Network Full-Body Detection (Class 0: Person).
2. Deteksi 360 Derajat: Menghadap depan, belakang (tampak punggung), samping, jalan, jongkok.
3. 100% Anti-Salah Kunci: Kursi, lemari, meja, dan dinding DIABAIKAN 100% (bukan manusia).
4. ByteTrack ID Locking: Mengunci 1 orang target secara konsisten.
5. Web HUD Server di Port 8080 (Bisa dibuka di browser PC/HP: http://localhost:8080).
6. Safe Display Fallback: Jika XCB / GUI Desktop error, sistem otomatis beralih ke Web HUD tanpa crash!
7. High-Speed Serial Engine ke ESP32: Mengirim koordinat X, Y, Jarak ke ESP32 secara gesit (50 Hz).
"""

import sys
import os

# Mencegah crash Qt XCB pada Linux Wayland / Pop!_OS
os.environ["QT_QPA_PLATFORM"] = "offscreen"

import time
import glob
import math
import argparse
import threading
import logging
import cv2

# Matikan log verbose Flask
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

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

# Cek & Import Flask
try:
    from flask import Flask, Response, render_template_string, jsonify, request
    HAS_FLASK = True
except ImportError:
    HAS_FLASK = False

# Resolusi Kamera Optimal untuk High FPS AI Tracking
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2   # 320
CENTER_Y = FRAME_HEIGHT // 2  # 240

# Asumsi Optik Kamera (Untuk estimasi jarak monokular manusia tinggi 1.7m)
FOCAL_LENGTH_PX = 580.0
HUMAN_REAL_HEIGHT_M = 1.70


import json

class GlobalVisionState:
    """State global thread-safe untuk Web HUD & Video Streaming."""
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.fps = 0.0
        self.status = "SCANNING_OWNER"
        self.target_id = None
        self.conf = 0.0
        self.distance_m = 0.0
        self.err_x = 0
        self.err_y = 0
        self.serial_status = "CONNECTING"
        self.serial_port = "None"
        self.reset_requested = False
        self.reset_ai_requested = False
        self.has_target = False
        self.ai_iteration = 0
        self.ai_loss = 0.0
        self.ai_best_loss = 999.0
        self.ai_reward = 85
        self.ai_smooth = 82
        self.ai_status = "TRAINING..."

vision_state = GlobalVisionState()


class ContinuousServoLearner:
    """
    Agen Pembelajaran Berkelanjutan (Real-Time Online Auto-Tuner):
    Mengamati deviasi posisi Anda dari tengah frame, kecepatan gerakan Anda,
    dan osilasi servo setiap episode (45 frame / ~1.5 detik).
    Secara otomatis melatih dan mengadaptasi parameter kehalusan, sensitivitas,
    dan deadzone agar kamera selalu mempertahankan posisi Anda di tengah
    secara super mulus (butter-smooth) tanpa sentakan dan tanpa delay!
    """
    def __init__(self, config_path="servo_learned_ai.json"):
        self.config_path = config_path
        self.best_loss = 999.0
        self.iteration = 0
        self.status = "TRAINING REALTIME..."
        self.metrics_str = "Mengumpulkan Data..."
        self.reward_pct = 85

        # Parameter yang dilatih secara adaptif
        self.learned_speed_pan = 42       # Responsivitas Pan horizontal (%)
        self.learned_speed_tilt = 24      # Responsivitas Tilt vertikal (%)
        self.learned_smooth = 82          # Kehalusan filter EMA (82% sangat halus)
        self.learned_deadzone_x = 8       # Deadzone horizontal (8 px)
        self.learned_deadzone_y = 10      # Deadzone vertikal (10 px)

        # Buffer pengumpul metrik per-episode
        self.frame_count = 0
        self.sum_abs_x = 0.0
        self.sum_abs_y = 0.0
        self.sum_d_x = 0.0
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.overshoot_x = 0
        self.overshoot_y = 0
        self.last_sign_x = 0
        self.last_sign_y = 0
        self.episode_start = time.time()

        self.load_learned_params()

    def load_learned_params(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, "r") as f:
                    data = json.load(f)
                    self.learned_speed_pan = int(data.get("speed_pan", 42))
                    self.learned_speed_tilt = int(data.get("speed_tilt", 24))
                    self.learned_smooth = int(data.get("smooth", 82))
                    self.learned_deadzone_x = int(data.get("deadzone_x", 8))
                    self.learned_deadzone_y = int(data.get("deadzone_y", 10))
                    self.best_loss = float(data.get("best_loss", 999.0))
                    self.iteration = int(data.get("iteration", 0))
                    print(f"[AI LEARNER] Memuat model tersimpan: Iterasi #{self.iteration}, Best Loss: {self.best_loss:.1f}, Kehalusan: {self.learned_smooth}%")
        except Exception as e:
            print(f"[AI LEARNER] Info load: {e}")

    def save_learned_params(self):
        try:
            data = {
                "speed_pan": int(self.learned_speed_pan),
                "speed_tilt": int(self.learned_speed_tilt),
                "smooth": int(self.learned_smooth),
                "deadzone_x": int(self.learned_deadzone_x),
                "deadzone_y": int(self.learned_deadzone_y),
                "best_loss": float(round(self.best_loss, 2)),
                "iteration": int(self.iteration),
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            with open(self.config_path, "w") as f:
                json.dump(data, f, indent=2)
            print(f"[AI LEARNER] Pembelajaran tersimpan! Iterasi #{self.iteration}, Best Loss: {self.best_loss:.1f} | Kelancaran: {self.reward_pct}%")
        except Exception as e:
            print(f"[AI LEARNER] Gagal simpan: {e}")

    def step(self, err_x, err_y):
        now = time.time()
        abs_x = abs(err_x)
        abs_y = abs(err_y)
        d_x = abs(err_x - self.prev_err_x)

        sign_x = 1 if err_x > 4 else (-1 if err_x < -4 else 0)
        sign_y = 1 if err_y > 5 else (-1 if err_y < -5 else 0)

        if self.last_sign_x != 0 and sign_x != 0 and sign_x != self.last_sign_x:
            self.overshoot_x += 1
        if self.last_sign_y != 0 and sign_y != 0 and sign_y != self.last_sign_y:
            self.overshoot_y += 1

        self.last_sign_x = sign_x
        self.last_sign_y = sign_y
        self.prev_err_x = err_x
        self.prev_err_y = err_y

        self.sum_abs_x += abs_x
        self.sum_abs_y += abs_y
        self.sum_d_x += d_x
        self.frame_count += 1

        # Evaluasi episode setiap ~45 frame (~1.5 detik)
        if self.frame_count >= 45 or (now - self.episode_start > 1.6):
            mae_x = self.sum_abs_x / max(1, self.frame_count)
            mae_y = self.sum_abs_y / max(1, self.frame_count)
            jitter_x = self.sum_d_x / max(1, self.frame_count)
            over_x = self.overshoot_x
            over_y = self.overshoot_y

            loss = round(0.30 * mae_x + 0.20 * mae_y + 10.0 * over_x + 14.0 * over_y + 0.25 * jitter_x, 1)
            self.reward_pct = max(10, min(99, int(100 - loss)))
            self.iteration += 1

            # Adaptasi Heuristik: Halus, Gesit, dan Menjaga di Tengah
            if over_x >= 2:
                self.learned_smooth = min(92, self.learned_smooth + 2)
                self.learned_speed_pan = max(25, self.learned_speed_pan - 3)
                self.learned_deadzone_x = min(16, self.learned_deadzone_x + 1)
            elif mae_x > 35:
                self.learned_speed_pan = min(65, self.learned_speed_pan + 3)
                self.learned_smooth = max(68, self.learned_smooth - 1)
                self.learned_deadzone_x = max(4, self.learned_deadzone_x - 1)

            if over_y >= 2:
                self.learned_speed_tilt = max(15, self.learned_speed_tilt - 2)
                self.learned_deadzone_y = min(18, self.learned_deadzone_y + 1)
            elif mae_y > 40:
                self.learned_speed_tilt = min(40, self.learned_speed_tilt + 2)

            self.status = f"TRAINING #{self.iteration}"
            self.metrics_str = f"Loss: {loss:.1f} | Smooth: {self.learned_smooth}%"

            if loss < self.best_loss:
                self.best_loss = loss
                self.save_learned_params()

            with vision_state.lock:
                vision_state.ai_iteration = self.iteration
                vision_state.ai_loss = loss
                vision_state.ai_best_loss = self.best_loss
                vision_state.ai_reward = self.reward_pct
                vision_state.ai_smooth = self.learned_smooth
                vision_state.ai_status = self.status

            # Reset akumulator episode
            self.frame_count = 0
            self.sum_abs_x = 0.0
            self.sum_abs_y = 0.0
            self.sum_d_x = 0.0
            self.overshoot_x = 0
            self.overshoot_y = 0
            self.episode_start = now


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
    """Pengirim paket serial non-blocking ke ESP32 dengan rate-limiting ~50 Hz."""
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
        while self.running:
            now = time.time()
            if self.ser is None:
                if now - last_reconnect > 2.0:
                    last_reconnect = now
                    port_to_open = self.port or find_serial_port()
                    if port_to_open:
                        try:
                            self.ser = serial.Serial(port_to_open, self.baud, timeout=0.01, write_timeout=0.01)
                            self.status = f"TX_OK ({port_to_open})"
                            with vision_state.lock:
                                vision_state.serial_status = "TX_OK"
                                vision_state.serial_port = port_to_open
                            print(f"\n[SERIAL] Berhasil terhubung ke ESP32 pada: {port_to_open}")
                        except Exception:
                            self.ser = None
                            self.status = "DISCONNECTED"
                            with vision_state.lock:
                                vision_state.serial_status = "DISCONNECTED"
            else:
                try:
                    with self.lock:
                        msg_to_send = self.latest_packet

                    self.ser.write(msg_to_send.encode('utf-8'))
                except Exception:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    self.status = "RECONNECTING"
                    with vision_state.lock:
                        vision_state.serial_status = "RECONNECTING"

            time.sleep(0.02)  # ~50 Hz

    def close(self):
        self.running = False
        if self.ser:
            try:
                self.ser.write(b"LOST\n")
                self.ser.close()
            except Exception:
                pass


HTML_PAGE = """<!DOCTYPE html>
<html lang="id">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Xiaomi / DJI Style AI Follower HUD</title>
    <style>
        :root {
            --bg-color: #0d1117;
            --card-bg: #161b22;
            --neon-green: #00ff88;
            --neon-cyan: #00d9ff;
            --neon-amber: #ffaa00;
            --neon-red: #ff3366;
            --text-color: #c9d1d9;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-color);
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 15px;
            min-height: 100vh;
        }
        .header {
            width: 100%;
            max-width: 900px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-bottom: 15px;
            border-bottom: 1px solid #30363d;
            margin-bottom: 15px;
        }
        .header h1 {
            font-size: 1.4rem;
            color: var(--neon-green);
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .badge-live {
            background: var(--neon-red);
            color: white;
            padding: 2px 8px;
            border-radius: 4px;
            font-size: 0.75rem;
            font-weight: bold;
            animation: pulse 1.5s infinite;
        }
        @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.4; } 100% { opacity: 1; } }
        .main-container {
            width: 100%;
            max-width: 900px;
            display: grid;
            grid-template-columns: 1fr;
            gap: 15px;
        }
        .video-box {
            background: #000;
            border-radius: 10px;
            overflow: hidden;
            border: 1px solid #30363d;
            position: relative;
            box-shadow: 0 8px 24px rgba(0,0,0,0.5);
            display: flex;
            justify-content: center;
        }
        .video-box img {
            width: 100%;
            height: auto;
            max-height: 540px;
            object-fit: contain;
            display: block;
        }
        .grid-cards {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
            gap: 12px;
        }
        .card {
            background: var(--card-bg);
            border: 1px solid #30363d;
            border-radius: 8px;
            padding: 12px 16px;
            display: flex;
            flex-direction: column;
            gap: 4px;
        }
        .card .title {
            font-size: 0.75rem;
            text-transform: uppercase;
            color: #8b949e;
            letter-spacing: 0.5px;
        }
        .card .value {
            font-size: 1.3rem;
            font-weight: bold;
            color: #fff;
        }
        .val-green { color: var(--neon-green) !important; }
        .val-cyan { color: var(--neon-cyan) !important; }
        .val-amber { color: var(--neon-amber) !important; }
        .btn-panel {
            display: flex;
            gap: 10px;
            margin-top: 10px;
        }
        .btn {
            background: #21262d;
            color: #c9d1d9;
            border: 1px solid #30363d;
            padding: 10px 18px;
            border-radius: 6px;
            cursor: pointer;
            font-weight: bold;
            transition: 0.2s;
        }
        .btn:hover { background: #30363d; color: #fff; }
        .btn-reset { border-color: var(--neon-amber); color: var(--neon-amber); }
        .btn-reset:hover { background: var(--neon-amber); color: #000; }
    </style>
</head>
<body>
    <div class="header">
        <h1><span>🎯</span> Xiaomi / DJI Style AI Follower <span class="badge-live">LIVE AI</span></h1>
        <div id="top-fps" style="font-size: 0.9rem; font-weight: bold; color: var(--neon-green);">FPS: --</div>
    </div>

    <div class="main-container">
        <div class="video-box">
            <img src="/video_feed" alt="Live AI Camera Feed">
        </div>

        <div class="grid-cards">
            <div class="card">
                <div class="title">Status Target</div>
                <div id="stat-status" class="value val-green">SCANNING...</div>
            </div>
            <div class="card">
                <div class="title">Jarak Monokular</div>
                <div id="stat-dist" class="value val-cyan">-- m</div>
            </div>
            <div class="card">
                <div class="title">Deviasi Horizontal (ErrX)</div>
                <div id="stat-errx" class="value">-- px</div>
            </div>
            <div class="card">
                <div class="title">Deviasi Vertikal (ErrY)</div>
                <div id="stat-erry" class="value">-- px</div>
            </div>
            <div class="card">
                <div class="title">Serial ESP32</div>
                <div id="stat-serial" class="value val-amber">CONNECTING</div>
            </div>
            <div class="card">
                <div class="title">AI Auto-Learner</div>
                <div id="stat-ai" class="value val-cyan">TRAINING...</div>
            </div>
        </div>

        <div class="btn-panel">
            <button class="btn btn-reset" onclick="resetTarget()">🔄 Reset Target (Kunci Ulang Orang Baru)</button>
        </div>
    </div>

    <script>
        function updateHUD() {
            fetch('/api/status')
                .then(r => r.json())
                .then(d => {
                    document.getElementById('top-fps').innerText = 'FPS: ' + d.fps.toFixed(1) + ' | Ryzen 5 Engine';
                    const stEl = document.getElementById('stat-status');
                    stEl.innerText = d.status;
                    if (d.has_target) {
                        stEl.className = 'value val-green';
                    } else {
                        stEl.className = 'value val-amber';
                    }
                    document.getElementById('stat-dist').innerText = d.distance_m > 0 ? d.distance_m.toFixed(2) + ' m' : '-- m';
                    document.getElementById('stat-errx').innerText = (d.err_x > 0 ? '+' : '') + d.err_x + ' px';
                    document.getElementById('stat-erry').innerText = (d.err_y > 0 ? '+' : '') + d.err_y + ' px';
                    const serEl = document.getElementById('stat-serial');
                    serEl.innerText = d.serial_status;
                    if (d.serial_status.startsWith('TX_OK')) {
                        serEl.className = 'value val-green';
                    } else {
                        serEl.className = 'value val-amber';
                    }
                    document.getElementById('stat-ai').innerText = '#' + d.ai_iteration + ' (' + d.ai_reward + '%)';
                })
                .catch(e => console.error(e));
        }

        function resetTarget() {
            fetch('/api/reset', { method: 'POST' })
                .then(r => r.json())
                .then(d => alert('Target berhasil di-reset! Berdirilah di depan kamera untuk mengunci ID baru.'))
                .catch(e => console.error(e));
        }

        setInterval(updateHUD, 200);
    </script>
</body>
</html>
"""


def create_app():
    """Membuat Flask Web App untuk streaming Web HUD."""
    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template_string(HTML_PAGE)

    @app.route('/video_feed')
    def video_feed():
        def generate():
            while True:
                with vision_state.lock:
                    jpeg = vision_state.latest_jpeg
                if jpeg is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
                time.sleep(0.033)  # ~30 FPS stream
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/api/status')
    def api_status():
        with vision_state.lock:
            return jsonify({
                "fps": vision_state.fps,
                "status": vision_state.status,
                "has_target": vision_state.has_target,
                "target_id": vision_state.target_id,
                "distance_m": vision_state.distance_m,
                "err_x": vision_state.err_x,
                "err_y": vision_state.err_y,
                "serial_status": vision_state.serial_status,
                "serial_port": vision_state.serial_port,
                "ai_iteration": vision_state.ai_iteration,
                "ai_loss": vision_state.ai_loss,
                "ai_best_loss": vision_state.ai_best_loss,
                "ai_reward": vision_state.ai_reward,
                "ai_smooth": vision_state.ai_smooth,
                "ai_status": vision_state.ai_status
            })

    @app.route('/api/reset', methods=['POST'])
    def api_reset():
        with vision_state.lock:
            vision_state.reset_requested = True
        return jsonify({"status": "OK"})

    return app


def draw_hud(frame, target_box, conf, track_id, distance_m, fps, serial_status, learner=None):
    """Menggambar HUD futuristik ala Xiaomi Smart Follower & DJI Gimbal."""
    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2

    # 1. Crosshair Tengah Horizon
    color_dim = (80, 80, 80)
    cv2.line(frame, (cx - 25, cy), (cx + 25, cy), color_dim, 1)
    cv2.line(frame, (cx, cy - 25), (cx, cy + 25), color_dim, 1)
    cv2.circle(frame, (cx, cy), 18, color_dim, 1)

    if target_box is not None:
        bx, by, bw, bh = target_box
        tcx = bx + bw // 2
        tcy = by + bh // 2
        color_locked = (0, 255, 120)  # Hijau neon

        # 2. Corner Brackets (Gaya DJI)
        corner_len = min(20, bw // 4, bh // 4)
        th = 2
        cv2.line(frame, (bx, by), (bx + corner_len, by), color_locked, th)
        cv2.line(frame, (bx, by), (bx, by + corner_len), color_locked, th)
        cv2.line(frame, (bx + bw, by), (bx + bw - corner_len, by), color_locked, th)
        cv2.line(frame, (bx + bw, by), (bx + bw, by + corner_len), color_locked, th)
        cv2.line(frame, (bx, by + bh), (bx + corner_len, by + bh), color_locked, th)
        cv2.line(frame, (bx, by + bh), (bx, by + bh - corner_len), color_locked, th)
        cv2.line(frame, (bx + bw, by + bh), (bx + bw - corner_len, by + bh), color_locked, th)
        cv2.line(frame, (bx + bw, by + bh), (bx + bw, by + bh - corner_len), color_locked, th)

        # 3. Garis Deviasi
        cv2.circle(frame, (tcx, tcy), 5, color_locked, -1)
        cv2.line(frame, (cx, cy), (tcx, tcy), (0, 200, 255), 1, cv2.LINE_AA)

        # 4. Badge Target
        id_str = f"ID #{track_id}" if track_id is not None else "TARGET"
        badge_text = f"LOCKED: {id_str} ({int(conf * 100)}%) | {distance_m:.2f}m"
        (tw_t, th_t), _ = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (bx, max(0, by - 24)), (bx + tw_t + 10, max(24, by)), (0, 255, 120), -1)
        cv2.putText(frame, badge_text, (bx + 5, max(17, by - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, "MENCARI TARGET MANUSIA (360)...", (cx - 180, cy - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA)

    # 5. Header Banner
    cv2.rectangle(frame, (0, 0), (w, 35), (20, 20, 20), -1)
    status_color = (0, 255, 0) if target_box is not None else (0, 165, 255)
    status_str = "TRACKING LOCKED (FULL-BODY 360)" if target_box is not None else "STANDBY SCANNING"
    cv2.putText(frame, f"AI FOLLOWER: {status_str}", (15, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2, cv2.LINE_AA)

    fps_text = f"FPS: {fps:4.1f} | Ryzen 5 AI"
    cv2.putText(frame, fps_text, (w - 220, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # 6. Footer Banner
    cv2.rectangle(frame, (0, h - 30), (w, h), (15, 15, 15), -1)
    if learner is not None:
        footer_text = f"Serial: {serial_status} | AI Train #{learner.iteration} ({learner.reward_pct}%) | Loss: {learner.best_loss:.1f} | Smooth: {learner.learned_smooth}%"
    else:
        footer_text = f"Serial: {serial_status} | Web HUD: http://localhost:8080"
    cv2.putText(frame, footer_text, (15, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="Xiaomi/DJI Style High-Precision Full-Body AI Follower")
    parser.add_argument("--camera", type=int, default=0, help="Indeks kamera USB (default: 0)")
    parser.add_argument("--port", type=str, default=None, help="Port serial ESP32 (contoh: /dev/ttyUSB0)")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="Bobot model YOLO (default: yolov8n.pt)")
    parser.add_argument("--conf", type=float, default=0.50, help="Ambang batas kepercayaan (default: 0.50)")
    parser.add_argument("--web-port", type=int, default=8080, help="Port Web HUD (default: 8080)")
    parser.add_argument("--gui", action="store_true", help="Buka jendela pop-up GUI desktop (jika display X11 aktif)")
    parser.add_argument("--dry-run", action="store_true", help="Jalankan simulasi tanpa kirim serial")
    args = parser.parse_args()

    print("=" * 65)
    print("   XIAOMI / DJI STYLE HIGH-PRECISION FULL-BODY AI FOLLOWER   ")
    print(f"   Web HUD : http://localhost:{args.web_port} (Buka di Browser!)")
    print("=" * 65)

    # 1. Jalankan Web Server Flask di Background Thread
    if HAS_FLASK:
        app = create_app()
        web_thread = threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=args.web_port, threaded=True, use_reloader=False),
            daemon=True
        )
        web_thread.start()
        print(f"[WEB] Web HUD aktif di port {args.web_port} -> \033[1;32mhttp://localhost:{args.web_port}\033[0m")
    else:
        print("[WARN] Library 'flask' tidak ditemukan. Web HUD dinonaktifkan.")

    # 2. Cek Ketersediaan YOLO
    if not HAS_YOLO:
        print("\n[ERROR FATAL] Library 'ultralytics' belum terinstal!")
        print("Jalankan: pip install ultralytics\n")
        return

    print(f"[*] Memuat model AI Full-Body: {args.model}...")
    try:
        model = YOLO(args.model)
        print("[OK] Model YOLOv8 berhasil dimuat!")
    except Exception as e:
        print(f"[ERROR] Gagal memuat model YOLO: {e}")
        return

    # 3. Inisialisasi Serial Transmitter ke ESP32
    serial_sender = FastSerialSender(port=args.port, baud=115200, dry_run=args.dry_run)
    print(f"[*] Status Serial ESP32: {serial_sender.status}")

    # 4. Buka Kamera USB
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
    print(f"[*] Buka browser Anda di: http://localhost:{args.web_port} untuk melihat live stream.")
    print("-" * 65)

    learner = ContinuousServoLearner()
    smooth_err_x = 0.0
    smooth_err_y = 0.0
    locked_track_id = None
    last_fps_time = time.time()
    last_terminal_print = 0.0
    frame_count = 0
    fps = 30.0
    gui_active = args.gui

    # Coba buat window GUI jika diminta (--gui)
    if gui_active:
        try:
            cv2.namedWindow("Xiaomi Style AI Follower (Full-Body 360)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Xiaomi Style AI Follower (Full-Body 360)", 800, 600)
        except Exception as e:
            gui_active = False
            print(f"[INFO] GUI desktop tidak dapat dibuka ({e}). Berjalan dalam mode WEB HUD murni.")

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

            # Cek jika ada reset dari Web HUD
            with vision_state.lock:
                if vision_state.reset_requested:
                    locked_track_id = None
                    vision_state.reset_requested = False
                    print("[RESET] Target ID di-reset via Web HUD.")

            # 5. Inferensi YOLOv8 dengan Pelacak ByteTrack Bawaan
            # classes=[0] HANYA mendeteksi Person (Manusia). Lemari & kursi DIABAIKAN 100%!
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

            # 6. Logika Penguncian Target Cerdas (ID Persistence)
            target_human = None
            if len(detected_humans) > 0:
                if locked_track_id is not None:
                    for h_obj in detected_humans:
                        if h_obj["id"] == locked_track_id:
                            target_human = h_obj
                            break

                if target_human is None:
                    detected_humans.sort(
                        key=lambda h_obj: math.hypot(h_obj["cx"] - CENTER_X, h_obj["cy"] - CENTER_Y)
                    )
                    target_human = detected_humans[0]
                    locked_track_id = target_human["id"]

            # 7. Hitung Deviasi & Kirim ke ESP32
            if target_human is not None:
                bx, by, bw, bh = target_human["box"]
                tcx, tcy = target_human["cx"], target_human["cy"]

                aim_cy = by + int(bh * 0.35)
                raw_err_x = tcx - CENTER_X
                raw_err_y = aim_cy - CENTER_Y

                # Step pembelajaran mandiri (Continuous Online Auto-Trainer)
                learner.step(raw_err_x, raw_err_y)

                # Filter Trajektori EMA Super Halus (Anti-Sentak & Menjaga di Tengah)
                alpha = max(0.12, min(0.38, 1.0 - (learner.learned_smooth / 100.0)))
                smooth_err_x += alpha * (raw_err_x - smooth_err_x)
                smooth_err_y += alpha * (raw_err_y - smooth_err_y)

                # Deadzone dinamis adaptif hasil training AI
                send_err_x = int(smooth_err_x) if abs(smooth_err_x) > learner.learned_deadzone_x else 0
                send_err_y = int(smooth_err_y) if abs(smooth_err_y) > learner.learned_deadzone_y else 0

                dist_m = (HUMAN_REAL_HEIGHT_M * FOCAL_LENGTH_PX) / max(20, bh)
                dist_m = max(0.5, min(8.0, dist_m))
                dist_cm = int(dist_m * 100)

                serial_sender.send(send_err_x, send_err_y, dist_cm)

                draw_hud(frame, target_human["box"], target_human["conf"],
                         target_human["id"], dist_m, fps, serial_sender.status, learner)

                with vision_state.lock:
                    vision_state.has_target = True
                    vision_state.status = f"LOCKED: ID #{target_human['id']} ({int(target_human['conf']*100)}%)"
                    vision_state.target_id = target_human['id']
                    vision_state.conf = target_human['conf']
                    vision_state.distance_m = dist_m
                    vision_state.err_x = send_err_x
                    vision_state.err_y = send_err_y
                    vision_state.fps = fps
            else:
                smooth_err_x = 0.0
                smooth_err_y = 0.0
                serial_sender.send_lost()
                draw_hud(frame, None, 0.0, None, 0.0, fps, serial_sender.status, learner)

                with vision_state.lock:
                    vision_state.has_target = False
                    vision_state.status = "SCANNING_OWNER..."
                    vision_state.target_id = None
                    vision_state.distance_m = 0.0
                    vision_state.err_x = 0
                    vision_state.err_y = 0
                    vision_state.fps = fps

            # Cetak Telemetri Realtime ke Terminal (4 kali per detik)
            if now - last_terminal_print >= 0.25:
                last_terminal_print = now
                ser_st = serial_sender.status
                if target_human is not None:
                    print(f"[LOCKED: ID #{target_human['id']} ({int(target_human['conf']*100)}%)] FPS: {fps:4.1f} | ErrX: {send_err_x:+4d} px | Jarak: {dist_m:4.2f} m | AI Train #{learner.iteration} ({learner.reward_pct}%) | Serial: {ser_st}")
                else:
                    print(f"[      SCANNING 360      ] FPS: {fps:4.1f} | Mencari Target Manusia...     | AI Train #{learner.iteration} | Serial: {ser_st}")

            # Encode JPEG untuk Web HUD
            ret_enc, jpeg_buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if ret_enc:
                with vision_state.lock:
                    vision_state.latest_jpeg = jpeg_buf.tobytes()

            # 8. Tampilkan di Window GUI jika tersedia
            if gui_active:
                try:
                    cv2.imshow("Xiaomi Style AI Follower (Full-Body 360)", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('r'):
                        locked_track_id = None
                        print("[RESET] Kunci target di-reset oleh keyboard.")
                except Exception:
                    gui_active = False
                    cv2.destroyAllWindows()
                    print("[INFO] Gagal me-render jendela GUI. Melanjutkan streaming di Web HUD: http://localhost:8080")

    except KeyboardInterrupt:
        print("\n[STOP] Program dihentikan pengguna.")
    finally:
        serial_sender.close()
        cap.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        print("[SHUTDOWN] Selesai.")


if __name__ == "__main__":
    main()
