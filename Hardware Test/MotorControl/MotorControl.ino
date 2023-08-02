/**********************************************************************************

   1. Date    : 2023-08-02
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno
   4. Purpose : PWM control on DC Motor
   5. Manual  : Speed and direction of the DC motor can be controlled by inputting
                the 'MotorDuty' value, which is PWM signal, into the serial monitor

 **********************************************************************************/

/*=============================================================================
  ===========================         Variables        =========================
  =============================================================================*/

// User input
int input = 0;
long index[3] = {0, 0, 0};
long index_Read = 0;

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
unsigned long RPMInterrupt[2] = {0, 0};   // 모터 속도 계산용 인터럽트 변수

// DC motor control
double MotorDuty[2] = {1, 1};             // PWM control (duty ratio, 0 to 100 %)
int CalculatedMotorDuty = 0;
unsigned int MotorDirection[2] = {0, 0};  // 모터 제어 (direction)
unsigned int MotorPeriod = 10;            // 모터 제어 (period, ms)
unsigned long MotorInterrupt[2] = {0, 0}; // 모터 제어용 인터럽트 변수

// Serial monitor display
unsigned long SerialInterrupt = 0;        // 시리얼 통신용 인터럽트 변수

/*=============================================================================
  ==========================       Arduino setup       =========================
  =============================================================================*/
void setup() {
  Serial.begin(9600);

  // Motor driver IN1, IN2, IN3, IN4 pin connection
  DDRB = 0b011011;
  DDRD = 0x00;
}


/*=============================================================================
  ===========================       Arduino loop       =========================
  =============================================================================*/
void loop() {
  // DC motor Encoder pin connection
  Encoder_A[0] = (PIND & 0b00000100);
  Encoder_B[0] = (PIND & 0b00001000);
  Encoder_A[1] = (PIND & 0b00010000);
  Encoder_B[1] = (PIND & 0b00100000);

  /* User inputs 'MotorDuty' value on serial monitor
    Input example
    30  -> 30% MotorDuty, CW
    030 -> 30% MotorDuty, CW
    130 -> 30% MotorDuty, CCW
  */
  input = Serial.read();
  if (input != -1) {    // '-1' means there is no input from the user

    // Read input data from the serial buffer until the user enters ';'  -> ';' is 59 on ASCII code
    if (input != 59) {
      index[index_Read] = input - 48;
      index_Read++;
    }

    // End of user input when he enters ';' -> input = 59
    else {
      // User input is a single-digit number
      if (index_Read == 1) {
        CalculatedMotorDuty = index[0];
      }

      // User input is a double-digit number
      else if (index_Read == 2) {
        CalculatedMotorDuty = 10 * index[0] + index[1];
        MotorDirection[0] = 0;
        MotorDirection[1] = 0;
      }

      // User input is a three-digit number
      else if (index_Read == 3) {
        CalculatedMotorDuty = 10 * index[1] + index[2];
        MotorDirection[0] = index[0];
        MotorDirection[1] = index[0];
      }
      index_Read = 0;

      MotorDuty[0] = CalculatedMotorDuty;
      MotorDuty[1] = CalculatedMotorDuty;

      Serial.print("Motor Duty = ");
      Serial.print(CalculatedMotorDuty);
      Serial.println("%");
      Serial.print("Motor Direction 1 = ");
      Serial.print(MotorDirection[0]);
      Serial.println();
      Serial.print("Motor Direction 2 = ");
      Serial.print(MotorDirection[1]);
      Serial.println();
    }
  }

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