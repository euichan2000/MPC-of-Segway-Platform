 /**********************************************************************************

   1. Date    : 2023-07-24
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno
   4. Purpose : Plotting MPU6050 sensor data
    - Roll angle (Raw)
    - Pitch angle (Raw)
    - Roll angle (LPF applied)
    - Pitch angle (LPF applied)
    - Roll angle (KF applied)
    - Pitch angle (KF applied)
  
   5. Notice  : KF tuning is not done yet
  
 **********************************************************************************/

#include <Wire.h>
#include <MPU6050.h>

/*=============================================================================
 ===========================         Variables        =========================
 =============================================================================*/
MPU6050 mpu;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double Roll_Raw, Pitch_Raw = 0.0;

// LPF 변수
double Roll_AlphaParameter = 0.7;  // Roll LPF parameter
double Pitch_AlphaParameter = 0.7; // Pitch LPF parameter
double Roll_LPF, Pitch_LPF = 0.0;

// Kalman Filter 변수
double Q_angle = 0.001;   // Process noise covariance for the accelerometer
double Q_gyro = 0.003;    // Process noise covariance for the gyro
double R_angle = 0.03;    // Measurement noise covariance
double Roll_KF = 0.0;  // The roll angle calculated by the Kalman Filter
double Roll_bias = 0.0;   // Gyro bias calculated by the Kalman Filter
double Pitch_KF = 0.0; // The pitch angle calculated by the Kalman Filter
double Pitch_bias = 0.0;  // Gyro bias calculated by the Kalman Filter
double P_00 = 0.0, P_01 = 0.0, P_10 = 0.0, P_11 = 0.0; // Error covariance matrix

/*=============================================================================
 ==========================       Arduino setup       =========================
 =============================================================================*/
void setup() {
  Serial.begin(9600);

  //PLX-DAQ 프로그램 사용을 위한
  Serial.println("CLEARDATA");
  Serial.println("Roll_RAW, Roll_LPF, Roll_Kalman, Pitch_RAW, Pitch_LPF, Pitch_Kalman");

  Wire.begin();
  mpu.initialize();
  while (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    delay(1000);
  }
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

/*=============================================================================
 ===========================       Kalman Filter      =========================
 =============================================================================*/
void KalmanFilterRoll(double newAngle, double newRate, double dt) {
  // Prediction Update
  Roll_KF += dt * (newRate - Roll_bias);
  P_00 += dt * (dt*P_11 - P_01 - P_10 + Q_angle);
  P_01 -= dt * P_11;
  P_10 -= dt * P_11;
  P_11 += Q_gyro * dt;

  // Measurement Update
  double y = newAngle - Roll_KF;
  double S = P_00 + R_angle;
  double K_0 = P_00 / S;
  double K_1 = P_10 / S;

  Roll_KF += K_0 * y;
  Roll_bias += K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
}

void KalmanFilterPitch(double newAngle, double newRate, double dt) {
  // Prediction Update
  Pitch_KF += dt * (newRate - Pitch_bias);
  P_00 += dt * (dt*P_11 - P_01 - P_10 + Q_angle);
  P_01 -= dt * P_11;
  P_10 -= dt * P_11;
  P_11 += Q_gyro * dt;

  // Measurement Update
  double y = newAngle - Pitch_KF;
  double S = P_00 + R_angle;
  double K_0 = P_00 / S;
  double K_1 = P_10 / S;

  Pitch_KF += K_0 * y;
  Pitch_bias += K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
}

/*=============================================================================
 ===========================       Arduino loop       =========================
 =============================================================================*/
void loop() {
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Calculate Roll angle & apply LPF and KF (angle unit : degree)
  Roll_Raw = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Roll_LPF = Roll_AlphaParameter * Roll_LPF + (1 - Roll_AlphaParameter) * Roll_Raw;
  KalmanFilterRoll(Roll_LPF, GyX / 131.0, 0.01); // GyX divided by 131 to convert to degrees per second

  // Calculate Pitch angle & apply LPF and KF (angle unit : degree)
  Pitch_Raw = atan2(AcY, sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Pitch_LPF = Pitch_AlphaParameter * Pitch_LPF + (1 - Pitch_AlphaParameter) * Pitch_Raw;
  KalmanFilterPitch(Pitch_LPF, GyY / 131.0, 0.01); // GyY divided by 131 to convert to degrees per second

  // Plot roll and pitch angles on Serial Plotter
  Serial.print(Roll_Raw);
  Serial.print(", ");
  Serial.print(Roll_LPF);
  Serial.print(", ");
  Serial.print(Roll_KF); // Kalman Filtered Roll angle
  Serial.print(", ");
  Serial.print(Pitch_Raw);
  Serial.print(", ");
  Serial.print(Pitch_LPF);
  Serial.print(", ");
  Serial.println(Pitch_KF); // Kalman Filtered Pitch angle

  delay(100);
}
