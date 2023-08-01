/*********************************************************************
  
   1. Date    : 2023-08-00
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

 *********************************************************************/

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
float Roll_Raw = 0.0;



// PID control
double Kp = 0.6;                  // P gain
double Ki = 0.0;                  // I gain
double Kd = 0.3;                  // D gain
double target_angle = 0.0;        // target angle of inverted pendulum
double actual_angle;              // control variable
double PID_control_value;         
double error_sum;
double error_previous = 0.0;
unsigned long previous_time = 0.0;  // Previous time for PID control

// DC motor velocity control
double MotorDuty[2] = {1, 1};             // 모터 제어 (duty ratio, 0 to 100 %)
int16_t MotorDuty_Int[2] = {0, 0};
unsigned int MotorDirection[2] = {0, 0};  // 모터 제어 (direction)

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

  // Calculate roll angle with MPU6050 sensor data
  Roll_Raw = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180.0 / PI;

  // Control variable
  actual_angle = Roll_Raw;

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

      MotorDuty[motorIndex] = abs(PID_control_value);           // Apply MotorDuty value
      if (PID_control_value > 0) MotorDirection[motorIndex] = 1;      // Rotate the motor in the forward direction
      else if (PID_control_value < 0) MotorDirection[motorIndex] = 0; // Rotate the motor in the reverse direction

      error_previous = error;
      previous_time = current_time;
    }
  }
}
