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
#define SLAVE_SELECT_PIN 10

/*=============================================================================
  ===========================         Variables        =========================
  =============================================================================*/
// SPI communication
int16_t ReceivedMotorDuty_Int[2];

// DC motor velocity control
unsigned long MotorInterrupt[2] = {0, 0}; // 모터 제어용 인터럽트 변수
unsigned int MotorDirection[2] = {0, 0};  // 모터 제어 (direction)
unsigned int MotorPeriod = 10;            // 모터 제어 (period, ms)

float MotorDuty[2] = {0.0, 0.0};

/*=============================================================================
  ==========================       Arduino setup       =========================
  =============================================================================*/
void setup() {
  // SPI communication setup
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  Serial.begin(9600);

  // DC motor pin setup
  DDRD = 0x00;
  DDRB = 0b001111;
}

ISR(SPI_STC_vect) {
  static byte motorIndex = 0;
  static byte highByte;
  static boolean isFirstByte = true;

  byte receivedByte = SPDR;

  if (isFirstByte) {
    highByte = receivedByte;
    isFirstByte = false;
  } else {
    isFirstByte = true;
    int16_t lowByte = receivedByte;
    ReceivedMotorDuty_Int[motorIndex] = ((int16_t)highByte << 8) | lowByte;

    // Convert the received integer back to a floating-point MotorDuty value
    MotorDuty[motorIndex] = ReceivedMotorDuty_Int[motorIndex] / 100.0;

    // Print the received value to the serial monitor
    Serial.print("SlaveReceive(MotorDuty[");
    Serial.print(motorIndex);
    Serial.print("]): ");
    Serial.println(MotorDuty[motorIndex]);

    motorIndex = (motorIndex + 1) % 2;
  }
}

/*=============================================================================
  ===========================       Arduino loop       =========================
  =============================================================================*/
void loop() {
  // Command both DC motors with their respective 'MotorDuty' values obtained from the master
  for (int motorIndex = 0; motorIndex < 2; motorIndex++) {
    if (micros() - MotorInterrupt[motorIndex] * MotorPeriod * 1000 >= MotorPeriod * 1000) {
      MotorInterrupt[motorIndex]++;

      // DC motor pin connection -> 8,9,11,12 (pin#10 is for SPI connection)
      PORTB = B00010010;
    }
    else {
      PORTB = B00001001;
    }
    if (micros() - MotorInterrupt[motorIndex] * MotorPeriod * 1000 >= MotorPeriod * 1000 * MotorDuty[motorIndex] / 100) {
      PORTB = B00000000;
    }
  }
}
