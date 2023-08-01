 /*********************************************************************
  
   1. Date    : 2023-07-24
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno
   4. Purpose : Plotting MPU6050 sensor data
    - Roll angle (Raw)
    - Pitch angle (Raw)
    - Roll angle (LPF applied)
    - Pitch angle (LPF applied)
  
 *********************************************************************/

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



/*=============================================================================
 ==========================       Arduino setup       =========================
 =============================================================================*/
void setup() {
  Serial.begin(9600);

  //PLX-DAQ 프로그램 사용을 위한
  Serial.println("CLEARDATA");
  Serial.println("Roll_RAW, Roll_LPF, Pitch_RAW, Pitch_LPF");

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
 ===========================       Arduino loop       =========================
 =============================================================================*/
void loop() {
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Calculate Roll angle & apply LPF (angle unit : degree)
  Roll_Raw = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Roll_LPF = Roll_AlphaParameter * Roll_LPF + (1 - Roll_AlphaParameter) * Roll_Raw;

  // Calculate Pitch angle & apply LPF (angle unit : degree)
  Pitch_Raw = atan2(AcY, sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / M_PI;
  Pitch_LPF = Pitch_AlphaParameter * Pitch_LPF + (1 - Pitch_AlphaParameter) * Pitch_Raw;

  // Plot rall angle and pitch angle on Serial Plotter
  Serial.print(Roll_Raw);
  Serial.print(", ");
  Serial.print(Roll_LPF);
  Serial.print(", ");
  Serial.print(Pitch_Raw);
  Serial.print(", ");
  Serial.println(Pitch_LPF);

  delay(100);
}
