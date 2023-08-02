/*
Date    :  2015-01-15
Maker   :  Lee SeungHyun (Blog Address : http://dkwltmdgus.blog.me)
MCU     :  Arduino Mega
Purpose :  9Byte / Real number SPI Commnuication Test
Please do not change this section
*/

#include <SPI.h>

void setup (void)
{
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  SPCR |= _BV(SPE);
  //SPCR |= _BV(DORD);
  SPCR &= ~_BV(MSTR);
  SPCR |= _BV(SPIE);

  SPI.setClockDivider(SPI_CLOCK_DIV16);

  Serial.begin(9600);
}

struct Axis {
  double rest;
  int multiple;
  int add;
} messenger;

boolean plus = false;
boolean minus = false;

boolean Check[9] = {0,};

ISR (SPI_STC_vect)
{
  byte Confirm = SPDR;

  if (Confirm == 0) {
    Check[0] = true;
    Check[1] = false;
    Check[2] = false;
    Check[3] = false;
    Check[4] = false;
    Check[5] = false;
    Check[6] = false;
    Check[7] = false;
    Check[8] = false;
  }
  else if (Confirm == 1) {
    Check[0] = false;
    Check[1] = true;
    Check[2] = false;
    Check[3] = false;
    Check[4] = false;
    Check[5] = false;
    Check[6] = false;
    Check[7] = false;
    Check[8] = false;
  }
  else if (Confirm == 2) {
    Check[0] = false;
    Check[1] = false;
    Check[2] = true;
    Check[3] = false;
    Check[4] = false;
    Check[5] = false;
    Check[6] = false;
    Check[7] = false;
    Check[8] = false;
  }
  else if (Confirm == 3) {
    Check[0] = false;
    Check[1] = false;
    Check[2] = false;
    Check[3] = true;
    Check[4] = false;
    Check[5] = false;
    Check[6] = false;
    Check[7] = false;
    Check[8] = false;
  }
  else if (Confirm == 4) {
    Check[0] = false;
    Check[1] = false;
    Check[2] = false;
    Check[3] = false;
    Check[4] = true;
    Check[5] = false;
    Check[6] = false;
    Check[7] = false;
    Check[8] = false;
  }
  else if (Confirm == 5) {
    Check[0] = false;
    Check[1] = false;
    Check[2] = false;
    Check[3] = false;
    Check[4] = false;
    Check[5] = true;
    Check[6] = false;
    Check[7] = false;
    Check[8] = false;
  }
  else if (Confirm == 6) {
    Check[0] = false;
    Check[1] = false;
    Check[2] = false;
    Check[3] = false;
    Check[4] = false;
    Check[5] = false;
    Check[6] = true;
    Check[7] = false;
    Check[8] = false;
  }
  else if (Confirm == 7) {
    Check[0] = false;
    Check[1] = false;
    Check[2] = false;
    Check[3] = false;
    Check[4] = false;
    Check[5] = false;
    Check[6] = false;
    Check[7] = true;
    Check[8] = false;
  }
   else if (Confirm == 8) {
    Check[0] = false;
    Check[1] = false;
    Check[2] = false;
    Check[3] = false;
    Check[4] = false;
    Check[5] = false;
    Check[6] = false;
    Check[7] = false;
    Check[8] = true;
  }
  
}

  double roll = 30.55;
  double pitch = 60.55;
  double yaw = -59.33;


  void loop (void)
  {
    struct Axis X_Axis;
    X_Axis = Sep(roll);

    struct Axis Y_Axis;
    Y_Axis = Sep(pitch);

    struct Axis Z_Axis;
    Z_Axis = Sep(yaw);
    /*
    SPI 규칙
    데이터 총 9byte
    통신 순서
    8 - 0 - 1 - 2 - 3 - 4 - 5 - 6 - 7 - 8
    */

    // Roll (X)
    if (Check[0] == true) {
      SPDR = X_Axis.add;
      Serial.print(X_Axis.add);
      Serial.print(" ");
      // Serial.print("x1");
      Check[0] = false;
    }
    else if (Check[1] == true) {
      //X_Axis.rest = X_Axis.rest * 100;
      int TempX= X_Axis.rest;
      SPDR = X_Axis.rest;
      Serial.print(X_Axis.rest);
      Serial.print(" ");
      // Serial.print("x2");
      Check[1] = false;
    }
    else if (Check[2] == true) {
      SPDR = X_Axis.multiple;
      Serial.print(X_Axis.multiple);
      Serial.print(" /");
      // Serial.println("x3");
      Check[2] = false;
    }

    // Pitch (Y)
    else if (Check[3] == true) {
      SPDR = Y_Axis.add;
      Serial.print(Y_Axis.add);
      Serial.print(" ");
      //Serial.print("y1");
      Check[3] = false;
    }
    else if (Check[4] == true) {
      //Y_Axis.rest = Y_Axis.rest * 100;
      int TempY= Y_Axis.rest;
      SPDR = Y_Axis.rest;
      Serial.print(Y_Axis.rest);
      Serial.print(" ");
      // Serial.print("y2");
      Check[4] = false;
    }
    else if (Check[5] == true) {
      SPDR = Y_Axis.multiple;
      Serial.print(Y_Axis.multiple);
      Serial.print(" /");
      // Serial.println("y3");
      Check[5] = false;
    }

    // Yaw (Z)
    else if (Check[6] == true) {
      SPDR = Z_Axis.add;
      Serial.print(Z_Axis.add);
      Serial.print(" ");
      // Serial.print("z1");
      Check[6] = false;
    }
    else if (Check[7] == true) {
      //Z_Axis.rest = Z_Axis.rest * 100;
      int TempZ= Z_Axis.rest;
      SPDR = Z_Axis.rest;
      Serial.print(Z_Axis.rest);
      Serial.print(" ");
      // Serial.print("z2");
      Check[7] = false;
    }
  else if (Check[8] == true) {
    SPDR = Z_Axis.multiple;
    Serial.print(Z_Axis.multiple);
    Serial.println(" ");
    Check[8] = false;
  }

  }

  // Data분리 함수 Sep = Separation
  struct Axis Sep(double value) {
    if (value == 0) {
      messenger.rest = 0;
      messenger.multiple = 0;
      messenger.add = 0;
      return messenger;
    }
    boolean add_Check = true;

    if (value > 0) {
      plus = true;
      minus = false;
    }

    else if (value < 0) {
      plus = false;
      minus = true;
    }

    value = abs(value);

    if (plus)
      messenger.rest = value - floor(value);
    else if (minus)
      messenger.rest = -(value - floor(value));

    value = floor(value);

    if (value >= 255) {
      if (plus)
        messenger.multiple = value / 255;
      else if (minus)
        messenger.multiple = -(value / 255);
    }

    else if (0 < value && value < 255) {
      messenger.multiple = 0;
      if (minus)
        value = -value;
    }

    for (int i = 0; i < abs(messenger.multiple); i++) {
      if (plus)
        messenger.add = value - (255 * messenger.multiple);
      else if (minus)
        messenger.add = -(value + (255 * messenger.multiple));
      add_Check = false;
    }
    if (add_Check)
      messenger.add = value;
    return messenger;
  }
