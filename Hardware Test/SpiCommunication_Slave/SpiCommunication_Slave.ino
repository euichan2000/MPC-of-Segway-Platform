 /**********************************************************************************

   1. Date    : 2023-07-31
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno (Slave board in Master-Slave system)
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

#define SLAVE_SELECT_PIN 10

int16_t receivedRoll_Int;

void setup() {
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);

  SPI.setClockDivider(SPI_CLOCK_DIV16);

  Serial.begin(9600);
}

ISR(SPI_STC_vect) {
  static byte highByte;
  static boolean isFirstByte = true;

  byte receivedByte = SPDR;

  if (isFirstByte) {
    highByte = receivedByte;
    isFirstByte = false;
  } else {
    isFirstByte = true;
    int16_t lowByte = receivedByte;
    receivedRoll_Int = ((int16_t)highByte << 8) | lowByte;

    // Convert the received integer back to a floating-point Roll_Raw value
    float receivedRoll_Raw = receivedRoll_Int / 100.0;

    // Print the received value to the serial monitor
    Serial.print("SlaveReceive(Roll_Raw): ");
    Serial.print(receivedRoll_Raw);
    Serial.println(" degree");
    Serial.println();
  }
}

void loop() {
  // The code in the loop function can be empty because SPI communication
  // is handled through the SPI interrupt.
}
