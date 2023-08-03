 /**********************************************************************************

   1. Date    : 2023-07-31
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno (Master board in Master-Slave system)
   4. Purpose : Setting master-slave system with SPI communication
   
   ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ                          ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ 
   |   Master  Board   |                         |   Slave   Board    |
   |   (Arduino Uno)   |                         |   (Arduino Uno)    |
   |                   | <--SPI communication--> |                    |
   | Read MPU6050 data |                         |                    |
   |         &         |                         |Receive MPU6050 data|
   |  Send MPU6050 data|                         |                    | 
   ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ                          ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

 **********************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float Roll_Raw = 0.0;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  pinMode(SS, OUTPUT);

  Wire.begin();
  mpu.initialize();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  Roll_Raw = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180.0 / PI;

  // Scale Roll_Raw to an integer value
  int16_t Roll_Int = Roll_Raw * 100;

  // Prepare the data to send to the slave
  byte lowByte = lowByte(Roll_Int);
  byte highByte = highByte(Roll_Int);

  // Start communication (activate slave select)
  digitalWrite(SS, LOW);

  // Send the data to the slave
  SPI.transfer(highByte);
  SPI.transfer(lowByte);

  // End communication (deactivate slave select)
  digitalWrite(SS, HIGH);

  // Print the values to the serial monitor
  Serial.print("MasterSend(Roll_Raw): ");
  Serial.print(Roll_Raw);
  Serial.println(" degree");
  Serial.println();

  // Delay for next iteration
  delay(100);
}
