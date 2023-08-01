/*********************************************************************
  
   1. Date    : 2023-07-31
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno
   4. Purpose : Balance control of inverted pendulum
   
                          ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ                         
                          | All-in-one Board  |                        
                          |   (Arduino Uno)   |
                          |                   | 
                          |  Balance control  |                      
                          |with MPU6050 sensor|
                          |         &         |
                          | Motor duty command|                  
                          ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ                       

 *********************************************************************/

#include <Wire.h>
#include <MPU6050.h>

/*=============================================================================
 ===========================         Variables        =========================
 =============================================================================*/
// MPU6050 variables
MPU6050 mpu;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double Roll_Raw, Pitch_Raw = 0.0;

// Moving Average Filter variables
const int numSamples = 10; // Number of samples to average
double Roll_Samples[numSamples]; // Array to hold Roll samples
double Pitch_Samples[numSamples]; // Array to hold Pitch samples
int sampleIndex = 0; // Current sample index

// Motor Control 변수
boolean Encoder_A[2] = {0, 0};
boolean Encoder_B[2] = {0, 0};
boolean Encoder_A_Before[2] = {0, 0};
boolean Encoder_B_Before[2] = {0, 0};
long Flag[2] = {0, 0};  // encoder 측정용 변수
long Flag_Read = 0; 
// int a = 0;
// int input = 0;
//long index[3] = {0, 0, 0};
//long degree[2] = {0, 0}; // 모터 각도
//long degree_temp[2] = {0, 0}; // 모터 각도(속도계산용 임시변수)
//double RPM[2] = {0, 0}; // 모터 속도 (RPM)

//unsigned long RPM_Interrupt[2] = {0, 0}; // 모터 속도 계산용 인터럽트 변수
//unsigned long Serial_Interrupt = 0; // 시리얼 통신용 인터럽트 변
unsigned long MotorInterrupt[2] = {0, 0}; // 모터 제어용 인터럽트 변수
double MotorDuty[2] = {1, 1}; // 모터 제어 (duty ratio, 0 to 100 %)
unsigned int MotorDirection[2] = {0, 0}; // 모터 제어 (direction)
unsigned int MotorPeriod = 10; // 모터 제어 (period, ms)

// PID Control 변수
double Kp = 0.6;                  // P gain
double Ki = 0.0;                  // I gain
double Kd = 0.3;                  // D gain
double TargetAngle = 0.0;        // target angle of inverted pendulum
double ActualAngle;              // control variable
double PID_control_value;         
double Error_Sum;
double Error_Previous = 0.0;
unsigned long PreviousTime = 0.0;  // Previous time for PID control

/*=============================================================================
 ==========================       Arduino setup       =========================
 =============================================================================*/
void setup(){
  Serial.begin(9600);

  // MPU6050 setting
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

  // DC motor pin connection
  DDRD = 0x00;
  DDRB = 0b001111;
}


/*=============================================================================
 ===========================       Arduino loop       =========================
 =============================================================================*/
void loop(){
  // get MPU6050 data
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Moving Average Filter for Roll
  Roll_Raw = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Roll_Samples[sampleIndex] = Roll_Raw;
  double Roll_LPF = calculateMovingAverage(Roll_Samples, numSamples);

  // Moving Average Filter for Pitch
  Pitch_Raw = atan2(AcY, sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Pitch_Samples[sampleIndex] = Pitch_Raw;
  double Pitch_LPF = calculateMovingAverage(Pitch_Samples, numSamples);

  sampleIndex++;
  if (sampleIndex >= numSamples) {
    sampleIndex = 0;
  }
  
  // Encoder connection
  Encoder_A[0] = (PIND & 0b00000100);
  Encoder_B[0] = (PIND & 0b00001000);
  Encoder_A[1] = (PIND & 0b00010000);
  Encoder_B[1] = (PIND & 0b00100000);

  for (int motorIndex = 0; motorIndex < 2; motorIndex++) {
    if ((Encoder_A[motorIndex] != Encoder_A_Before[motorIndex]) || (Encoder_B[motorIndex] != Encoder_B_Before[motorIndex]))
    {
      if (Encoder_A[motorIndex] != Encoder_B_Before[motorIndex]) // CW
      {
        Flag[motorIndex]++;
        MotorDirection[motorIndex] = 1;
      }
      else // CCW
      {
        Flag[motorIndex]--;
        MotorDirection[motorIndex] = 0;
      }
    }
    Encoder_A_Before[motorIndex] = Encoder_A[motorIndex];
    Encoder_B_Before[motorIndex] = Encoder_B[motorIndex];

    //degree[motorIndex] = Flag[motorIndex] * 360 / (52 * 60);
    
    // 추가 작성 내용: actual angle = MPU6050 센서에서 받아온 roll angle 값
    ActualAngle = Roll_LPF;
  }

  // Balancing control through PID
  PID_Control();
  Serial.println(ActualAngle);

  // Command DCmotor with 'MotorDuty[motorIndex]' value which obtained on PID control
  for (int motorIndex = 0; motorIndex < 2; motorIndex++) {
    if (micros() - MotorInterrupt[motorIndex] * MotorPeriod * 1000 >= MotorPeriod * 1000){
      MotorInterrupt[motorIndex]++;

      // DC motor pin connection -> 8,9,11,12 (pin#10 is for SPI connection)
      if (MotorDirection[motorIndex] == 0){
        PORTB = B00010010;
      }
      else{
        PORTB = B00001001;
      }
    }
    if (micros() - MotorInterrupt[motorIndex] * MotorPeriod * 1000 >= MotorPeriod * 1000 * MotorDuty[motorIndex] / 100) {
      PORTB = B00000000;
    }
  }
}

/*=============================================================================
 ===========================       PID control        =========================
 =============================================================================*/
void PID_Control(){
  unsigned long current_time = micros();
  double sampling_time = (double)(current_time - PreviousTime) / 1000.0; // Convert to milliseconds
  double Pcontrol[2] = {0, 0};
  double Icontrol[2] = {0, 0};
  double Dcontrol[2] = {0, 0};

  for (int motorIndex = 0; motorIndex < 2; motorIndex++) {
    if (sampling_time >= 20) { // Minimum sampling time in milliseconds
      double error = TargetAngle - ActualAngle;

      Error_Sum += error * sampling_time;
      if (Error_Sum > 255) Error_Sum = 255;
      else if (Error_Sum < -255) Error_Sum = -255;

      Pcontrol[motorIndex] = Kp * error;
      Icontrol[motorIndex] = Ki * Error_Sum;
      Dcontrol[motorIndex] = Kd * (error - Error_Previous) / sampling_time;

      // Calculate PID value
      PID_control_value = Pcontrol[motorIndex] + Icontrol[motorIndex] + Dcontrol[motorIndex];

      if (PID_control_value > 255) PID_control_value = 255;
      else if (PID_control_value < -255) PID_control_value = -255;

      MotorDuty[motorIndex] = abs(PID_control_value);           // Apply MotorDuty value
      if (PID_control_value > 0) MotorDirection[motorIndex] = 1;      // Rotate the motor in the forward direction
      else if (PID_control_value < 0) MotorDirection[motorIndex] = 0; // Rotate the motor in the reverse direction

      Error_Previous = error;
      PreviousTime = current_time;
    }
  }
}

// Function to calculate the moving average of an array
double calculateMovingAverage(double* samples, int numSamples) {
  double sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += samples[i];
  }
  return sum / numSamples;
}
