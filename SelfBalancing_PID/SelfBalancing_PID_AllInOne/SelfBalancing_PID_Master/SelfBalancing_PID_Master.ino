/**********************************************************************************

   1. Date    : 2023-08-02
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno (Master board in Master-Slave system)
   4. Purpose : Balacne control of inverted pendulum
   5. This code is reconstruction of the 'BalanceControl_AllInOne.ino'
      code in the form of Master-Slave system
      
   ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ                          ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ 
   |   Master  Board   |                         |   Slave   Board    |
   |   (Arduino Uno)   |                         |   (Arduino Uno)    |
   |                   | <--SPI communication--> |                    |
   |  Balance control  |                         |                    |
   |with MPU6050 sensor|                         | Motor duty commmand| 
   ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ                          ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

 **********************************************************************************/

#include <SPI.h>
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
double Roll_AlphaParameter = 0.9;  // Roll LPF 계수
double Pitch_AlphaParameter = 0.9; // Pitch LPF 계수
double Roll_LPF, Pitch_LPF = 0.0;

// DC motor velocity control
double MotorDuty[2] = {0, 0};             // 모터 제어 (duty ratio, 0 to 100 %)
unsigned int MotorDirection[2] = {0, 0};  // 모터 제어 (direction)

// PID Control
double Kp = 43;               // P gain
double Ki = 0;                // I gain
double Kd = 15;               // D gain
double TargetAngle = 0.0;     // target angle of inverted pendulum
double ActualAngle;           // control variable
double PID_value;
double Error_sum;
double Error_previous = 0.0;
unsigned long Time_previous = 0.0;  // Previous time for PID control

// SPI communication
int16_t MotorDuty_Int[2] = {0, 0};

/*=============================================================================
 ==========================       Arduino setup       =========================
 =============================================================================*/
void setup() {
  Serial.begin(9600);

  // SPI communication setup
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  pinMode(SS, OUTPUT);

  // MPU6050 I2C communication setup
  Wire.begin();
  mpu.initialize();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

}


/*=============================================================================
 ===========================       Arduino loop       =========================
 =============================================================================*/
void loop() {
  // Get MPU6050 sensor data
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Calcualate roll angle & apply LPF
  Roll_raw = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Roll_LPF = Roll_AlphaParameter * Roll_LPF + (1 - Roll_AlphaParameter) * Roll_raw;

  // Calcualate pitch angle & apply LPF
  Pitch_raw = atan2(AcY, sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Pitch_LPF = Pitch_AlphaParameter * Pitch_LPF + (1 - Pitch_AlphaParameter) * Pitch_raw;

  // PID control for segway balancing
  PID_Control();

  // Send MotorDuty value to Slave board
  for (int motorIndex = 0; motorIndex < 2; motorIndex++){
    // Scale MotorDuty to an integer value
    MotorDuty_Int[motorIndex] = MotorDuty[motorIndex] * 100;

    // Prepare the data to send to the slave
    byte lowByte = lowByte(MotorDuty_Int[motorIndex]);
    byte highByte = highByte(MotorDuty_Int[motorIndex]);

     // Start communication (activate slave select)
    digitalWrite(SS, LOW);

    // Send the data to the slave
    SPI.transfer(highByte);
    SPI.transfer(lowByte);

    // End communication (deactivate slave select)
    digitalWrite(SS, HIGH);

    // Print the values to the serial monitor
    Serial.print("MasterSend(MotorDuty[");
    Serial.print(motorIndex);
    Serial.print("]) : ");
    Serial.print(MotorDuty[motorIndex]);
    Serial.println();

    // Delay for next iteration
    delay(100);
  }
  Serial.println();
}


/*=============================================================================
  ===========================     PID controller      =========================
  =============================================================================*/
void PID_Control() {
  unsigned long Time_current = micros();
  double SamplingTime = (double)(Time_current - Time_previous) / 1000.0; // Convert to milliseconds
  double Pcontrol[2] = {0, 0};
  double Icontrol[2] = {0, 0};
  double Dcontrol[2] = {0, 0};

  ActualAngle = Roll_LPF;

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

    // MotorDuty value is on 0~100%
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
