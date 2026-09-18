/**
 * esp32_servo_tracking.ino
 * Firmware ESP32 - Pengendali Servo Pan-Tilt Berbasis Visual Serial
 *
 * Kompatibel dengan: Arduino IDE / PlatformIO
 * Library yang dibutuhkan: ESP32Servo (instal via Arduino Library Manager)
 *
 * Pinout ESP32:
 * - Servo Pan  (Horizontal) : GPIO 18
 * - Servo Tilt (Vertikal)   : GPIO 19
 * - Baud Rate Serial        : 115200 bps
 */

#include <ESP32Servo.h>

// Definisi Pin & Batasan Servo
const int PIN_SERVO_PAN  = 18;
const int PIN_SERVO_TILT = 19;

// Batasan Sudut Aman Servo (Mencegah gir servo macet / terbakar)
const float PAN_MIN  = 20.0;
const float PAN_MAX  = 160.0;
const float PAN_MID  = 90.0;

const float TILT_MIN = 50.0;
const float TILT_MAX = 130.0;
const float TILT_MID = 90.0;

// Parameter Kontrol
const int DEADZONE_PX   = 12;    // Toleransi agar servo tidak bergetar (jitter)
const float KP_PAN      = 0.04;  // Gain proporsional horizontal (atur kecepatan respons)
const float KP_TILT     = 0.03;  // Gain proporsional vertikal
const unsigned long TIMEOUT_MS = 500; // Timeout jika komunikasi serial terputus

Servo servoPan;
Servo servoTilt;

float currentPan  = PAN_MID;
float currentTilt = TILT_MID;

unsigned long lastPacketTime = 0;
bool targetLost = true;

void setup() {
  Serial.begin(115200);
  
  // Konfigurasi Timer & Pin Servo untuk ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoPan.setPeriodHertz(50);   // Standard 50Hz servo SG90
  servoTilt.setPeriodHertz(50);

  servoPan.attach(PIN_SERVO_PAN, 500, 2400);   // Pulse width standar SG90
  servoTilt.attach(PIN_SERVO_TILT, 500, 2400);

  // Set posisi awal di tengah (90 derajat)
  servoPan.write((int)currentPan);
  servoTilt.write((int)currentTilt);

  Serial.println("[ESP32] Sistem Servo Tracking Siap. Menunggu data serial dari PC...");
}

void loop() {
  // 1. Parsing Serial Non-Blocking
  if (Serial.available() > 0) {
    String packet = Serial.readStringUntil('\n');
    packet.trim();

    if (packet == "LOST") {
      targetLost = true;
    } 
    else if (packet.startsWith("X:") && packet.indexOf(",Y:") != -1) {
      int idxX = packet.indexOf("X:");
      int idxY = packet.indexOf(",Y:");

      int err_x = packet.substring(idxX + 2, idxY).toInt();
      int err_y = packet.substring(idxY + 3).toInt();

      targetLost = false;
      lastPacketTime = millis();

      // 2. Kendali Proporsional Servo Pan (Horizontal)
      // Jika target di kanan (err_x > 0), kamera harus menoleh ke kanan
      // (Sesuaikan tanda minus/plus tergantung arah orientasi pemasangan servo Anda)
      if (abs(err_x) > DEADZONE_PX) {
        float deltaPan = - (err_x * KP_PAN);
        currentPan += deltaPan;
        currentPan = constrain(currentPan, PAN_MIN, PAN_MAX);
        servoPan.write((int)currentPan);
      }

      // 3. Kendali Proporsional Servo Tilt (Vertikal)
      // Jika target di bawah (err_y > 0), kamera harus menunduk
      if (abs(err_y) > DEADZONE_PX) {
        float deltaTilt = (err_y * KP_TILT);
        currentTilt += deltaTilt;
        currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);
        servoTilt.write((int)currentTilt);
      }
    }
  }

  // 4. Safety Fail-Safe (Timeout Handler)
  if (!targetLost && (millis() - lastPacketTime > TIMEOUT_MS)) {
    targetLost = true;
    // Pilihan: Biarkan di sudut terakhir atau perlahan kembali ke tengah
    // currentPan = PAN_MID;
    // servoPan.write(PAN_MID);
  }

  delay(5); // Loop interval kecil
}
