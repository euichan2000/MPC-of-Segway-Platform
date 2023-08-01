/*********************************************************************
  
   1. Date    : 2023-07-17
   2. Maker   : Janguk Kim
   3. MCU     : Arduino Uno
   4. Purpose : Motor speed control with MotorDuty value

 *********************************************************************/

/*=============================================================================
 ===========================         Variables        =========================
 =============================================================================*/
boolean Encoder_A[2] = {0, 0};
boolean Encoder_B[2] = {0, 0};
boolean Encoder_A_Before[2] = {0, 0};
boolean Encoder_B_Before[2] = {0, 0};
long flag[2] = {0, 0}; // encoder 측정용 변수
long flag_Read = 0; //
int a = 0;
int input = 0;
long index[3] = {0, 0, 0};
long degree[2] = {0, 0}; // 모터 각도
long degree_temp[2] = {0, 0}; // 모터 각도(속도계산용 임시변수)
double RPM[2] = {0, 0}; // 모터 속도 (RPM)
unsigned long RpmInterrupt[2] = {0, 0}; // 모터 속도 계산용 인터럽트 변수
unsigned long SerialInterrupt = 0; // 시리얼 통신용 인터럽트 변수
unsigned long MotorInterrupt[2] = {0, 0}; // 모터 제어용 인터럽트 변수
double MotorDuty[2] = {1, 1}; // 모터 제어 (duty ratio, 0 to 100 %)
unsigned int MotorDirection[2] = {0, 0}; // 모터 제어 (direction)
unsigned int MotorPeriod = 10; // 모터 제어 (period, ms)


/*=============================================================================
 ==========================       Arduino setup       =========================
 =============================================================================*/
void setup(){
  Serial.begin(9600);
  DDRD = 0x00;
  DDRB = 0b001111;
}


/*=============================================================================
 ===========================       Arduino loop       =========================
 =============================================================================*/
void loop() {
  Encoder_A[0] = (PIND & 0b00000100);
  Encoder_B[0] = (PIND & 0b00001000);
  Encoder_A[1] = (PIND & 0b00010000);
  Encoder_B[1] = (PIND & 0b00100000);

  for (int motorIndex = 0; motorIndex < 2; motorIndex++) {
    if ((Encoder_A[motorIndex] != Encoder_A_Before[motorIndex]) || (Encoder_B[motorIndex] != Encoder_B_Before[motorIndex]))
    {
      if (Encoder_A[motorIndex] != Encoder_B_Before[motorIndex]) // CW
      {
        flag[motorIndex]++;
        // MotorDirection[motorIndex] = 0;
      }
      else // CCW
      {
        flag[motorIndex]--;
        // MotorDirection[motorIndex] = 1;
      }
    }
    Encoder_A_Before[motorIndex] = Encoder_A[motorIndex];
    Encoder_B_Before[motorIndex] = Encoder_B[motorIndex];

    degree[motorIndex] = flag[motorIndex] * 360 / (52 * 60);
  }

  input = Serial.read();
  if (input != -1) {
    if (input != 59) {
      index[flag_Read] = input - 48;
      flag_Read++;
    }
    else {
      if (flag_Read == 1) {
        a = index[0];
      }
      else if (flag_Read == 2) {
        a = 10 * index[0] + index[1];
        MotorDirection[0] = 0;
        MotorDirection[1] = 0;
      }
      else if (flag_Read == 3) {
        a = 10 * index[1] + index[2];
        MotorDirection[0] = index[0];
        MotorDirection[1] = index[1];
      }
      flag_Read = 0;
      Serial.print("\n Motor Duty = ");
      Serial.print(a);
      Serial.print("   Motor Direction 1 = ");
      Serial.print(MotorDirection[0]);
      Serial.print("   Motor Direction 2 = ");
      Serial.print(MotorDirection[1]);
      MotorDuty[0] = a;
      MotorDuty[1] = a;
    }
  }

  for (int motorIndex = 0; motorIndex < 2; motorIndex++) {
    if (millis() - RpmInterrupt[motorIndex] * 100 >= 100) {
      RpmInterrupt[motorIndex]++;
      RPM[motorIndex] = (degree[motorIndex] - degree_temp[motorIndex]) * 600 / 360;
      degree_temp[motorIndex] = degree[motorIndex];
    }

    if (millis() - SerialInterrupt * 500 >= 500) {
      SerialInterrupt++;
      Serial.print("\nMotor ");
      Serial.print(motorIndex + 1);
      Serial.print(" Direction = ");
      if (MotorDirection[motorIndex] == 0) {
        Serial.print("CW");
      } else {
        Serial.print("CCW");
      }
      Serial.print("\tDegree = ");
      Serial.print(degree[motorIndex]);
      Serial.print("\tRPM = ");
      Serial.print(RPM[motorIndex]);
      Serial.println();
    }

    if (micros() - MotorInterrupt[motorIndex] * MotorPeriod * 1000 >= MotorPeriod * 1000) {
      MotorInterrupt[motorIndex]++;
      if (MotorDirection[motorIndex] == 0) {
        PORTB |= (1 << (motorIndex * 2 + 1)); // Motor 1, 2를 CW 방향으로 설정
        PORTB &= ~(1 << (motorIndex * 2));
      } else {
        PORTB &= ~(1 << (motorIndex * 2 + 1)); // Motor 1, 2를 CCW 방향으로 설정
        PORTB |= (1 << (motorIndex * 2));
      }
    }

    if (micros() - MotorInterrupt[motorIndex] * MotorPeriod * 1000 >= MotorPeriod * 1000 * MotorDuty[motorIndex] / 100) {
      PORTB &= ~(1 << (motorIndex * 2 + 1)); // Motor 1, 2를 정지 상태로 설정
      PORTB &= ~(1 << (motorIndex * 2));
    }
  }
}
