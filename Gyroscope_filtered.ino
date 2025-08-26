#include <Arduino_LSM9DS1.h>
#include <math.h>

struct Kalman {
  float Q_angle;
  float Q_bias;
  float R_measure;

  float angle;
  float bias;
  float rate;
  float P[2][2];
};

void initKalman(Kalman &kf) {
  kf.Q_angle = 0.005f;  // faster correction
  kf.Q_bias = 0.003f;
  kf.R_measure = 0.02f;  // trust accel more

  kf.angle = 0.0f;
  kf.bias = 0.0f;
  kf.P[0][0] = kf.P[0][1] = kf.P[1][0] = kf.P[1][1] = 0.0f;
}

float getAngle(Kalman &kf, float newAngle, float newRate, float dt) {
  kf.rate = newRate - kf.bias;
  kf.angle += dt * kf.rate;

  kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + kf.Q_angle);
  kf.P[0][1] -= dt * kf.P[1][1];
  kf.P[1][0] -= dt * kf.P[1][1];
  kf.P[1][1] += kf.Q_bias * dt;

  float y = newAngle - kf.angle;
  float S = kf.P[0][0] + kf.R_measure;
  float K0 = kf.P[0][0] / S;
  float K1 = kf.P[1][0] / S;

  kf.angle += K0 * y;
  kf.bias += K1 * y;

  float P00_temp = kf.P[0][0];
  float P01_temp = kf.P[0][1];

  kf.P[0][0] -= K0 * P00_temp;
  kf.P[0][1] -= K0 * P01_temp;
  kf.P[1][0] -= K1 * P00_temp;
  kf.P[1][1] -= K1 * P01_temp;

  return kf.angle;
}

Kalman kalmanPitch, kalmanRoll;
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  initKalman(kalmanPitch);
  initKalman(kalmanRoll);

  lastTime = micros();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Convert raw accel to angles
    float rollAcc = atan2(ay, az) * 180.0 / PI;
    float pitchAcc = atan(-ax / sqrt(ay * ay + az * az)) * 180.0 / PI;

    // Gyro in rad/s
    gx = gx * PI / 180.0;
    gy = gy * PI / 180.0;

    // Delta time
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    // Kalman filter update
    float pitch = getAngle(kalmanPitch, pitchAcc, gy, dt);
    float roll = getAngle(kalmanRoll, rollAcc, gx, dt);

    // Compute roll and pitch from accelerometer
    float raw_roll  = atan2(ay, az) * 180 / PI;
    float raw_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
    
    // Print at 20 Hz with short format
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {
      Serial.print("Roll: ");
      Serial.print(raw_roll);
      Serial.print(", ");
      Serial.print(roll);
      Serial.print(" | Pitch: ");
      Serial.print(raw_pitch);
      Serial.print(", ");
      Serial.println(pitch);
      lastPrint = millis();
    }
  }
}
