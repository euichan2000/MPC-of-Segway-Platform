/**********************************************************************************
   
   1. Date    : 2023-08-03
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno
   4. Purpose : Self-balancing of segway platform
   5. Warning : PID gain tuning is done. Please do not change the code!!

                          ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
                          | All-in-one Board  |
                          |   (Arduino Uno)   |
                          |                   |
                          |  Balance control  |
                          |with MPU6050 sensor|
                          |         &         |
                          | Motor duty command|
                          ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


    MPU6050 파일 보고 KF부분 다시 작성!!!
 **********************************************************************************/

#include <Wire.h>
#include <MPU6050.h>

/*=============================================================================
  ===========================         Variables        =========================
  =============================================================================*/
// MPU6050
MPU6050 mpu;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double Roll_raw, Pitch_raw = 0.0;

// Low Pass Filter
double Roll_AlphaParameter = 0.9;  // Roll LPF parameter
double Pitch_AlphaParameter = 0.9; // Pitch LPF parameter
double Roll_LPF, Pitch_LPF = 0.0;

// Encoder
boolean Encoder_A[2] = {0, 0};
boolean Encoder_B[2] = {0, 0};
boolean Encoder_A_Before[2] = {0, 0};
boolean Encoder_B_Before[2] = {0, 0};
long EncoderCount[2] = {0, 0};

// DC motor information
long Degree[2] = {0, 0};
long Degree_temp[2] = {0, 0};
double RPM[2] = {0, 0};
unsigned long RPMInterrupt[2] = {0, 0};   // Interrupt variable for motor RPM calculation

// DC motor control
double MotorDuty[2] = {1, 1};             // PWM motor control (duty ratio, 0 to 100 %)
int CalculatedMotorDuty = 0;
unsigned int MotorDirection[2] = {0, 0};  // PWM motor control (direction)
unsigned int MotorPeriod = 20;            // PWM motor control (period, ms)
unsigned long MotorInterrupt[2] = {0, 0}; // Interrupt variable for PWM motor control

// PID Control
double Kp = 150;              // P gain (Best fit: 150)
double Ki = 0;                // I gain (Not used)
double Kd = 5;                // D gain (Best fit: 5)
double TargetAngle = 2.5;     // Reference (Best fit: 2.5 degree, 0 degree was rather difficult to control the balance)
double ActualAngle;           // control feedback
double PID_value;
double Error_sum;
double Error_previous = 0.0;
unsigned long Time_previous = 0.0;  // Previous time for PID controller

// Kalman Filter
double Q_angle = 0.001; // Process noise covariance for the accelerometer
double Q_gyro = 0.003;  // Process noise covariance for the gyroscope
double R_angle = 0.03;  // Measurement noise covariance for the accelerometer
double x_bias = 0.0;    // Gyro bias
double XP_00, XP_01, XP_10, XP_11 = 0.0;
double YP_00, YP_01, YP_10, YP_11 = 0.0;
double KFangleX, KFangleY = 0.0;

/*=============================================================================
  ==========================       Arduino setup       =========================
  =============================================================================*/
void setup() {
  Serial.begin(9600);

  // MPU6050 setup
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

  // Motor driver IN1, IN2, IN3, IN4 pin connection (Arduino pin#8, pin#9, pin#11, pint#12)
  DDRB = 0b011011;
  DDRD = 0x00;
}

/*=============================================================================
  ===========================       Arduino loop       =========================
  =============================================================================*/
void loop() {
  // Get MPU6050 data
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Calcualate roll angle & apply LPF
  Roll_raw = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Roll_LPF = Roll_AlphaParameter * Roll_LPF + (1 - Roll_AlphaParameter) * Roll_raw;

  // Calcualate pitch angle & apply LPF
  Pitch_raw = atan2(AcY, sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Pitch_LPF = Pitch_AlphaParameter * Pitch_LPF + (1 - Pitch_AlphaParameter) * Pitch_raw;

  // Kalman Filter update
  double dt = 0.001; // Sampling time in seconds
  KFangleX = kalmanFilterX(Roll_LPF, GyX, dt);
  KFangleY = kalmanFilterY(Pitch_LPF, GyY, dt);

  // DC motor Encoder pin connection
  Encoder_A[0] = (PIND & 0b00000100);
  Encoder_B[0] = (PIND & 0b00001000);
  Encoder_A[1] = (PIND & 0b00010000);
  Encoder_B[1] = (PIND & 0b00100000);

  // Self balancing with PID controller
  PID_Control(KFangleY);

  // PWM control on DC motor with 'MotorDuty' value
  for (int motorIndex = 0; motorIndex < 2; motorIndex++) {
    if (micros() - MotorInterrupt[motorIndex] * MotorPeriod * 1000 >= MotorPeriod * 1000) {
      MotorInterrupt[motorIndex]++;

      // DC motor pin connection -> 8,9,11,12 (pin#10 is for SPI connection)
      if (MotorDirection[motorIndex] == 0) { // CW
        PORTB = B00010010;
      }
      else if (MotorDirection[motorIndex] == 1) { // CCW
        PORTB = B00001001;
      }
    }
    if (micros() - MotorInterrupt[motorIndex] * MotorPeriod * 1000 >= MotorPeriod * 1000 * MotorDuty[motorIndex] / 100) {
      PORTB = B00000000;  // Stop -> PWM control
    }
  }
}

/*=============================================================================
  ===========================     PID controller      =========================
  =============================================================================*/
void PID_Control(double angle) {
  unsigned long Time_current = micros();
  double SamplingTime = (double)(Time_current - Time_previous) / 1000.0; // Convert to milliseconds
  double Pcontrol[2] = {0, 0};
  double Icontrol[2] = {0, 0};
  double Dcontrol[2] = {0, 0};
  
  ActualAngle = angle;

  if (SamplingTime >= 10) { // Minimum sampling time in milliseconds
    double Error = TargetAngle - ActualAngle;

    Error_sum += Error * SamplingTime;
    if (Error_sum > 2000) Error_sum = 2000;
    else if (Error_sum < -2000) Error_sum = -2000;

    Pcontrol[0] = Kp * Error;
    Icontrol[0] = Ki * Error_sum;
    Dcontrol[0] = Kd * (Error - Error_previous) / SamplingTime;

    // Calculate PID value
    PID_value = Pcontrol[0] + Icontrol[0] + Dcontrol[0];

    if (PID_value > 2000) PID_value = 2000;
    else if (PID_value < -2000) PID_value = -2000;

    // Map the MotorDuty value to 0~100%
    MotorDuty[0] = map(abs(PID_value), 0, 2000, 0, 100);      
    MotorDuty[1] = map(abs(PID_value), 0, 2000, 0, 100);      

    if (PID_value > 0) {
      MotorDirection[0] = 1;      // Rotate the motor in the forward direction
      MotorDirection[1] = 1;      // Rotate the motor in the forward direction
    }
    else if (PID_value < 0) {
      MotorDirection[0] = 0;      // Rotate the motor in the forward direction
      MotorDirection[1] = 0;      // Rotate the motor in the forward direction
    }

    Error_previous = Error;
    Time_previous = Time_current;
  }
}

/*=============================================================================
  ===========================     Kalman Filter      ============================
  =============================================================================*/
double kalmanFilterX(double newAngle, double newRate, double dt) {
  // Prediction update
  KFangleX += dt * (newRate - x_bias);

  // Prediction error covariance update
  XP_00 += -dt * (XP_10 + XP_01) + Q_angle * dt;
  XP_01 += -dt * XP_11;
  XP_10 += -dt * XP_11;
  XP_11 += +Q_gyro * dt;

  // Compute the Kalman gain
  double y = newAngle - KFangleX;
  double S = XP_00 + R_angle;
  double K_0 = XP_00 / S;
  double K_1 = XP_10 / S;

  // Correction update
  KFangleX += K_0 * y;
  x_bias += K_1 * y;

  // Correction error covariance update
  double P00_temp = XP_00;
  double P01_temp = XP_01;

  XP_00 -= K_0 * P00_temp;
  XP_01 -= K_0 * P01_temp;
  XP_10 -= K_1 * P00_temp;
  XP_11 -= K_1 * P01_temp;

  return KFangleX;
}

double kalmanFilterY(double newAngle, double newRate, double dt) {
  // Prediction update
  KFangleY += dt * (newRate - x_bias);

  // Prediction error covariance update
  YP_00 += -dt * (YP_10 + YP_01) + Q_angle * dt;
  YP_01 += -dt * YP_11;
  YP_10 += -dt * YP_11;
  YP_11 += +Q_gyro * dt;

  // Compute the Kalman gain
  double y = newAngle - KFangleY;
  double S = YP_00 + R_angle;
  double K_0 = YP_00 / S;
  double K_1 = YP_10 / S;

  // Correction update
  KFangleY += K_0 * y;
  x_bias += K_1 * y;

  // Correction error covariance update
  double P00_temp = YP_00;
  double P01_temp = YP_01;

  YP_00 -= K_0 * P00_temp;
  YP_01 -= K_0 * P01_temp;
  YP_10 -= K_1 * P00_temp;
  YP_11 -= K_1 * P01_temp;

  return KFangleY;
}
