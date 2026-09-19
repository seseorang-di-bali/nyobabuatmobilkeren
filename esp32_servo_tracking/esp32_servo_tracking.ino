/**
 * esp32_servo_tracking.ino
 * Firmware ESP32 - Pengendali Servo Pan-Tilt & Sensor Laser VL53L0X (Bench Test)
 * Bagian dari Proyek Autonomous Vision-Guided Following Robot
 *
 * Kompatibel dengan: Arduino IDE / PlatformIO
 *
 * Library yang dibutuhkan (Pasang via Arduino IDE Library Manager):
 * 1. "ESP32Servo" oleh Kevin Harrington
 * 2. "Adafruit_VL53L0X" oleh Adafruit
 *
 * Wiring Pinout ESP32:
 * -------------------------------------------------------------
 * 1. Servo Pan  (Horizontal) : Signal -> GPIO 18 (VCC 5V, GND)
 * 2. Servo Tilt (Vertikal)   : Signal -> GPIO 19 (VCC 5V, GND)
 * 3. Sensor Laser VL53L0X (I2C):
 *    - VIN / VCC : 3.3V (atau 5V sesuai modul)
 *    - GND       : GND
 *    - SDA       : GPIO 21
 *    - SCL       : GPIO 22
 * 4. Komunikasi Serial ke STB/PC : USB Serial (Baud Rate 115200 bps)
 * -------------------------------------------------------------
 *
 * Protokol Serial yang Diterima:
 * - "X:<err_x>,Y:<err_y>,D:<dist_cm>\n" (Tracking aktif dengan estimasi jarak)
 * - "X:<err_x>,Y:<err_y>\n" (Tracking aktif tanpa jarak)
 * - "LOST\n" (Target terlepas)
 */

#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_VL53L0X.h>

// ================= PIN & BATASAN SERVO =================
const int PIN_SERVO_PAN  = 18;
const int PIN_SERVO_TILT = 19;

// Batasan Sudut Aman Servo (Mencegah gir SG90/MG996R mentok / rusak)
const float PAN_MIN  = 20.0;
const float PAN_MAX  = 160.0;
const float PAN_MID  = 90.0;

const float TILT_MIN = 60.0;  // Batasi agar tidak mendongak/menunduk terlalu ekstrem
const float TILT_MAX = 120.0;
const float TILT_MID = 90.0;

// Parameter Kendali PD Cerdas (Proportional + Derivative Electronic Damping)
// Mengeliminasi osilasi / overshooting saat diam, serta mengunci target di tengah dengan tenang
const int   DEADZONE_PAN_PX   = 12;    // Zona tenang horizontal: di dalam +/- 12px servo DIAM TOTAL
const int   DEADZONE_TILT_PX  = 14;    // Zona tenang vertikal: di dalam +/- 14px tilt DIAM TOTAL
const float KP_PAN            = 0.0070;// Penguatan proporsional horizontal stabil untuk rate 50 Hz
const float KD_PAN            = 0.0050;// Rem elektronik instan (Derivative Damping) saat mendekati tengah
const float KP_TILT           = 0.0040;// Penguatan proporsional vertikal
const float KD_TILT           = 0.0030;// Rem elektronik vertikal
const float MAX_STEP_PAN      = 1.2;   // Batas pergerakan per paket (derajat) - halus bebas sentak
const float MAX_STEP_TILT     = 0.8;   // Batas pergerakan vertikal per paket
const unsigned long SERIAL_TIMEOUT_MS = 600; // Timeout jika komunikasi terputus

// Konfigurasi Arah Putaran Servo (Invert jika mekanik servo terpasang terbalik)
const bool INVERT_PAN   = false; // Set true jika servo belok berlawanan arah target
const bool INVERT_TILT  = false; // Set true jika servo mendongak/menunduk terbalik

// ================= OBJEK PERANGKAT =================
Servo servoPan;
Servo servoTilt;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ================= VARIABEL GLOBAL =================
float targetPanAngle  = PAN_MID;
float targetTiltAngle = TILT_MID;

float currentPan  = PAN_MID;
float currentTilt = TILT_MID;

// Kecepatan peluncuran trajektori (EMA Smooth Factor)
// Nilai 0.22 memberikan sensasi "glide" halus seperti gimbal sinematik
const float TRAJECTORY_SMOOTH_FACTOR = 0.22;

inline int angleToMicros(float deg) {
  return (int)(500.0 + (deg / 180.0) * 1900.0);
}

int targetErrX = 0;
int targetErrY = 0;
int prevTargetErrX = 0;
int prevTargetErrY = 0;
int cameraDistCm = 0;

int laserDistCm = -1;
bool hasLaserSensor = false;

bool targetLocked = false;
unsigned long lastPacketTime = 0;
unsigned long lastTelemetryPrint = 0;

// Status Aksi Sasis Mobil (Simulasi Bench Test)
enum RobotAction {
  ACT_EMERGENCY_STOP,
  ACT_STOP_ZONE_TENANG,
  ACT_MAJU_MENGEJAR,
  ACT_MUNDUR_MENJAUH,
  ACT_SEARCHING
};
RobotAction currentAction = ACT_SEARCHING;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n==================================================");
  Serial.println("   AUTONOMOUS ROBOT: ESP32 VISION & TOF CONTROLLER");
  Serial.println("==================================================");

  // 1. Inisialisasi Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);

  servoPan.attach(PIN_SERVO_PAN, 500, 2400);
  servoTilt.attach(PIN_SERVO_TILT, 500, 2400);

  servoPan.write((int)currentPan);
  servoTilt.write((int)currentTilt);
  Serial.println("[SERVO] Pan (GPIO 18) & Tilt (GPIO 19) Siap pada 90 Derajat.");

  // 2. Inisialisasi Bus I2C & Sensor Laser VL53L0X
  Wire.begin(21, 22); // SDA = 21, SCL = 22
  Serial.print("[LASER] Mendeteksi Sensor VL53L0X pada I2C... ");
  if (lox.begin()) {
    hasLaserSensor = true;
    Serial.println("OK! Sensor Laser Aktif.");
  } else {
    hasLaserSensor = false;
    Serial.println("TIDAK DITEMUKAN! (Fallback menggunakan jarak kamera).");
  }

  Serial.println("[SYSTEM] Menunggu paket serial dari STB / PC...");
  Serial.println("--------------------------------------------------\n");
}

char rxBuffer[64];
int rxIndex = 0;

void processPacket(char* pkt) {
  if (strcmp(pkt, "LOST") == 0) {
    targetLocked = false;
    targetErrX = 0;
    targetErrY = 0;
    prevTargetErrX = 0;
    prevTargetErrY = 0;
    cameraDistCm = 0;
    return;
  }

  if (pkt[0] == 'X' && pkt[1] == ':') {
    char* idxY = strstr(pkt, ",Y:");
    char* idxD = strstr(pkt, ",D:");
    if (idxY != NULL) {
      *idxY = '\0';
      targetErrX = atoi(pkt + 2);
      if (idxD != NULL) {
        *idxD = '\0';
        targetErrY = atoi(idxY + 3);
        cameraDistCm = atoi(idxD + 3);
      } else {
        targetErrY = atoi(idxY + 3);
        cameraDistCm = 0;
      }

      targetLocked = true;
      lastPacketTime = millis();

      // KENDALI PD CERDAS DENGAN ELECTRONIC DAMPING (ANTI-OVERSHOOT)
      // 1. Pan (Horizontal): Hitung langkah sudut dengan rem derivatif
      if (abs(targetErrX) <= DEADZONE_PAN_PX) {
        // Target berada di zona tenang tengah -> servo diam 100% tanpa jitter
        prevTargetErrX = targetErrX;
      } else {
        float dirPan = INVERT_PAN ? 1.0 : -1.0;
        float dErrX = (float)(targetErrX - prevTargetErrX);
        // Rumus PD: KP * Error + KD * Derivatif (Rem aktif saat dErr berlawanan tanda)
        float deltaPan = dirPan * ((targetErrX * KP_PAN) + (dErrX * KD_PAN));
        deltaPan = constrain(deltaPan, -MAX_STEP_PAN, MAX_STEP_PAN);
        targetPanAngle += deltaPan;
        targetPanAngle = constrain(targetPanAngle, PAN_MIN, PAN_MAX);
        prevTargetErrX = targetErrX;
      }

      // 2. Tilt (Vertikal): Hitung langkah sudut vertikal dengan rem derivatif
      if (abs(targetErrY) <= DEADZONE_TILT_PX) {
        prevTargetErrY = targetErrY;
      } else {
        float dirTilt = INVERT_TILT ? -1.0 : 1.0;
        float dErrY = (float)(targetErrY - prevTargetErrY);
        float deltaTilt = dirTilt * ((targetErrY * KP_TILT) + (dErrY * KD_TILT));
        deltaTilt = constrain(deltaTilt, -MAX_STEP_TILT, MAX_STEP_TILT);
        targetTiltAngle += deltaTilt;
        targetTiltAngle = constrain(targetTiltAngle, TILT_MIN, TILT_MAX);
        prevTargetErrY = targetErrY;
      }
    }
  }
}

void loop() {
  // 1. INTERPOLASI GERAKAN SERVO 100 Hz (SUPER BUTTER-SMOOTH, KELAS GIMBAL DJI)
  static unsigned long lastSmoothTick = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastSmoothTick >= 10) {
    lastSmoothTick = nowMs;

    float diffPan = targetPanAngle - currentPan;
    if (abs(diffPan) > 0.02) {
      currentPan += diffPan * TRAJECTORY_SMOOTH_FACTOR;
      servoPan.writeMicroseconds(angleToMicros(currentPan));
    }

    float diffTilt = targetTiltAngle - currentTilt;
    if (abs(diffTilt) > 0.02) {
      currentTilt += diffTilt * TRAJECTORY_SMOOTH_FACTOR;
      servoTilt.writeMicroseconds(angleToMicros(currentTilt));
    }
  }

  // 2. BACA PAKET SERIAL 100% NON-BLOCKING (Zero-Timeout)
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxIndex > 0) {
        rxBuffer[rxIndex] = '\0';
        processPacket(rxBuffer);
        rxIndex = 0;
      }
    } else if (rxIndex < (int)sizeof(rxBuffer) - 1) {
      rxBuffer[rxIndex++] = c;
    }
  }

  // 3. FAILSAFE TIMEOUT SERIAL
  if (targetLocked && (millis() - lastPacketTime > SERIAL_TIMEOUT_MS)) {
    targetLocked = false;
  }

  // 4. BACA SENSOR LASER VL53L0X (Jika Terpasang)
  if (hasLaserSensor) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) { // Status 4 = out of range
      laserDistCm = measure.RangeMilliMeter / 10;
    } else {
      laserDistCm = -1; // Out of range (> 1.5 - 2m)
    }
  }

  // 5. TENTUKAN JARAK EFEKTIF (Sensor Fusion: Prioritas Laser, Fallback Kamera)
  int effectiveDistance = -1;
  String distSource = "NONE";

  if (hasLaserSensor && laserDistCm > 0 && laserDistCm < 200) {
    effectiveDistance = laserDistCm;
    distSource = "LASER";
  } else if (cameraDistCm > 0) {
    effectiveDistance = cameraDistCm;
    distSource = "KAMERA";
  }

  // 6. LOGIKA KEPUTUSAN KENDALI SASIS (SESUAI PLAN_AI_MOBIL.md)
  if (!targetLocked) {
    currentAction = ACT_SEARCHING;
  }
  else if (effectiveDistance > 0 && effectiveDistance < 30) {
    // Safety Cutoff Layer Mutlak: Jarak < 30 cm potong daya motor seketika!
    currentAction = ACT_EMERGENCY_STOP;
  }
  else if (effectiveDistance >= 35 && effectiveDistance <= 60) {
    // Jarak 35 - 60 cm: Mundur perlahan menjauh
    currentAction = ACT_MUNDUR_MENJAUH;
  }
  else if (effectiveDistance >= 70 && effectiveDistance <= 90) {
    // Jarak 70 - 90 cm: Zona tenang (Stop aman)
    currentAction = ACT_STOP_ZONE_TENANG;
  }
  else if (effectiveDistance > 100) {
    // Jarak > 100 cm: Maju mengejar proporsional
    currentAction = ACT_MAJU_MENGEJAR;
  }

  // 7. TELEMETRI MONITOR BENCH TEST (Hanya aktif untuk debugging serial monitor)
  // Dinonaktifkan saat live dengan STB agar buffer UART tetap bersih dan 0 latency
  #if 0
  if (millis() - lastTelemetryPrint > 1000) {
    lastTelemetryPrint = millis();
    Serial.printf("[ESP32] Pan:%d Tilt:%d Jarak:%d\n", (int)currentPan, (int)currentTilt, effectiveDistance);
  }
  #endif
}
