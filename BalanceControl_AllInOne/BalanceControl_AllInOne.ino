/*********************************************************************
  
   1. Date    : 2023-07-31
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno
   4. Purpose : Balacne control of inverted pendulum
   
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
// MPU6050 변수
MPU6050 mpu;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double Roll_Raw, Pitch_Raw = 0.0;


// Low Pass Filter 변수
double Roll_AlphaParameter = 0.9;  // Roll LPF 계수
double Pitch_AlphaParameter = 0.7; // Pitch LPF 계수
double Roll_LPF, Pitch_LPF = 0.0;


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
unsigned long Motor_Interrupt[2] = {0, 0}; // 모터 제어용 인터럽트 변수
double Motor_Duty[2] = {1, 1}; // 모터 제어 (duty ratio, 0 to 100 %)
unsigned int Motor_Dir[2] = {0, 0}; // 모터 제어 (direction)
unsigned int Motor_Period = 10; // 모터 제어 (period, ms)


// PID Control 변수
double Kp = 0.6;                  // P gain
double Ki = 0.0;                  // I gain
double Kd = 0.3;                  // D gain
double target_angle = 0.0;        // target angle of inverted pendulum
double actual_angle;              // control variable
double PID_control_value;         
double error_sum;
double error_previous = 0.0;
unsigned long previous_time = 0.0;  // Previous time for PID control


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

  // LPF on Roll
  Roll_Raw = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Roll_LPF = Roll_AlphaParameter * Roll_LPF + (1 - Roll_AlphaParameter) * Roll_Raw;

  // LPF on Pitch
  Pitch_Raw = atan2(AcY, sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Pitch_LPF = Pitch_AlphaParameter * Pitch_LPF + (1 - Pitch_AlphaParameter) * Pitch_Raw;
  
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
        Motor_Dir[motorIndex] = 1;
      }
      else // CCW
      {
        Flag[motorIndex]--;
        Motor_Dir[motorIndex] = 0;
      }
    }
    Encoder_A_Before[motorIndex] = Encoder_A[motorIndex];
    Encoder_B_Before[motorIndex] = Encoder_B[motorIndex];

    //degree[motorIndex] = Flag[motorIndex] * 360 / (52 * 60);
    
    // 추가 작성 내용: actual angle = MPU6050 센서에서 받아온 roll angle 값
    actual_angle = Roll_LPF;
  }

  // Balancing control through PID
  PID_Control();
  Serial.println(actual_angle);

  // Command DCmotor with 'Motor_Duty[motorIndex]' value which obtained on PID control
  for (int motorIndex = 0; motorIndex < 2; motorIndex++) {
    if (micros() - Motor_Interrupt[motorIndex] * Motor_Period * 1000 >= Motor_Period * 1000){
      Motor_Interrupt[motorIndex]++;
      if (Motor_Dir[motorIndex] == 0){
        PORTB |= (1 << (motorIndex * 2 + 1)); // Motor 1, 2를 CW 방향으로 설정
        PORTB &= ~(1 << (motorIndex * 2));
      }
      else{
        PORTB &= ~(1 << (motorIndex * 2 + 1)); // Motor 1, 2를 CCW 방향으로 설정
        PORTB |= (1 << (motorIndex * 2));
      }
    }
    if (micros() - Motor_Interrupt[motorIndex] * Motor_Period * 1000 >= Motor_Period * 1000 * Motor_Duty[motorIndex] / 100) {
      PORTB &= ~(1 << (motorIndex * 2 + 1)); // Motor 1, 2를 정지 상태로 설정
      PORTB &= ~(1 << (motorIndex * 2));
    }
  }

}

/*=============================================================================
 ===========================       PID control        =========================
 =============================================================================*/
void PID_Control(){
  unsigned long current_time = micros();
  double sampling_time = (double)(current_time - previous_time) / 1000.0; // Convert to milliseconds
  double Pcontrol[2] = {0, 0};
  double Icontrol[2] = {0, 0};
  double Dcontrol[2] = {0, 0};

  for (int motorIndex = 0; motorIndex < 2; motorIndex++) {
    if (sampling_time >= 20) { // Minimum sampling time in milliseconds
      double error = target_angle - actual_angle;

      error_sum += error * sampling_time;
      if (error_sum > 255) error_sum = 255;
      else if (error_sum < -255) error_sum = -255;

      Pcontrol[motorIndex] = Kp * error;
      Icontrol[motorIndex] = Ki * error_sum;
      Dcontrol[motorIndex] = Kd * (error - error_previous) / sampling_time;

      // Calculate PID value
      PID_control_value = Pcontrol[motorIndex] + Icontrol[motorIndex] + Dcontrol[motorIndex];

      if (PID_control_value > 255) PID_control_value = 255;
      else if (PID_control_value < -255) PID_control_value = -255;

      Motor_Duty[motorIndex] = abs(PID_control_value);           // Apply Motor_Duty value
      if (PID_control_value > 0) Motor_Dir[motorIndex] = 1;      // Rotate the motor in the forward direction
      else if (PID_control_value < 0) Motor_Dir[motorIndex] = 0; // Rotate the motor in the reverse direction

      error_previous = error;
      previous_time = current_time;
    }
  }
}
