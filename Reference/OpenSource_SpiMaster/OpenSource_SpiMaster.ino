
/*
Date    :  2015-01-15
Maker   :  Lee SeungHyun (Blog Address : http://dkwltmdgus.blog.me)
MCU     :  Arduino Mega
Purpose :  9Byte / Real number SPI Commnuication Test
Please do not change this section


SPI 규칙 
데이터 총 9byte
통신 순서
8 - 0 - 1 - 2 - 3 - 4 - 5 - 6 - 7

0, 1, 2 = roll(x)
3, 4, 5 = pitch(y)
6, 7, 8 = yaw(z)

First = 구조체명.add
Second = 구조체명.rest
Third = 구조체명.multiple
*/

#include <SPI.h>

void setup (void)
{
  SPI.begin ();
  digitalWrite(SS, HIGH);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  Serial.begin(9600);
}

struct Axis {
  double rest;
  int multiple;
  int add;
} messenger;

boolean A_Check = true;
boolean B_Check = false;
boolean C_Check = false;
boolean D_Check = false;
boolean E_Check = false;
boolean F_Check = false;
boolean G_Check = false;
boolean H_Check = false;
boolean I_Check = false;

double X[3] = {0,};
double Y[3] = {0,};
double Z[3] = {0,};

// double pitch;
// double roll;
// double yaw;

void loop (void) {
  
  digitalWrite(SS,LOW);
  if (A_Check) {
    //digitalWrite(SS, LOW);
    Z[2] = SPI.transfer(0);
    A_Check = false;
    B_Check = true;
    //Serial.println("0");
    delayMicroseconds(900);
    //digitalWrite(SS, HIGH);
  }
  if (B_Check) {
    //digitalWrite(SS, LOW);
    X[0] = SPI.transfer(1);
    B_Check = false;
    C_Check = true;
    //Serial.println("1");
    delayMicroseconds(900);
    //digitalWrite(SS, HIGH);
  }
  if (C_Check) {
    //digitalWrite(SS, LOW);
    X[1] = SPI.transfer(2);
    C_Check = false;
    D_Check = true;
    //Serial.println("2");
    delayMicroseconds(900);
    //digitalWrite(SS, HIGH);
  }
  if (D_Check) {
    //digitalWrite(SS, LOW);
    X[2] = SPI.transfer(3);
    D_Check = false;
    E_Check = true;
    //Serial.println("3");
    delayMicroseconds(900);
    //digitalWrite(SS, HIGH);
  }
  if (E_Check) {
    //digitalWrite(SS, LOW);
    Y[0] = SPI.transfer(4);
    E_Check = false;
    F_Check = true;
    //Serial.println("4");
    delayMicroseconds(900);
    //digitalWrite(SS, HIGH);
  }
  if (F_Check) {
    //digitalWrite(SS, LOW);
    Y[1] = SPI.transfer(5);
    F_Check = false;
    G_Check = true;
    //Serial.println("5");
    delayMicroseconds(900);
    //digitalWrite(SS, HIGH);
  }
  if (G_Check) {
    //digitalWrite(SS, LOW);
    Y[2] = SPI.transfer(6);
    G_Check = false;
    H_Check = true;
    //Serial.println("6");
    delayMicroseconds(900);
    //digitalWrite(SS, HIGH);
  }
  if (H_Check) {
    //digitalWrite(SS, LOW);
    Z[0] = SPI.transfer(7);
    H_Check = false;
    I_Check = true;
    //Serial.println("7");
    delayMicroseconds(900);
    //digitalWrite(SS, HIGH);
  }
  if (I_Check) {
    //digitalWrite(SS, LOW);
    Z[1] = SPI.transfer(8);
    I_Check = false;
    A_Check = true;
    delayMicroseconds(900);
   
    //Serial.println("8");
    //digitalWrite(SS, HIGH);
  }
  digitalWrite(SS, HIGH);
  
  
  // struct Axis X_Axis;
  // X_Axis.add = X[0];
  // X[1] = (double)X[1];
  // X[1] = X[1] - X[1]*0.99;
  // X_Axis.rest = X[1];
  // X_Axis.multiple = X[2];
  // roll = Trans(X_Axis);
  
  // struct Axis Y_Axis;
  // Y_Axis.add = Y[0];
  // Y[1] = (double)Y[1];
  // Y[1] = Y[1] - Y[1]*0.99;
  // Y_Axis.rest = Y[1];
  // Y_Axis.multiple = Y[2];
  // pitch = Trans(Y_Axis);
  
  // struct Axis Z_Axis;
  // Z_Axis.add = Z[0];
  // Z[1] = (double)Z[1];
  // Z[1] = Z[1] - Z[1]*0.99;
  // Z_Axis.rest = Z[1];
  // Z_Axis.multiple = Z[2];
  // yaw = Trans(Z_Axis);
 
  Serial.print(X[0]); Serial.print(" / ");
  Serial.print(X[1]); Serial.print(" / ");
  Serial.print(X[2]); Serial.print(" / ");
  //Serial.print(roll); 
  //Serial.print(" / ");

  Serial.print(Y[0]); Serial.print(" / ");
  Serial.print(Y[1]); Serial.print(" / ");
  Serial.print(Y[2]); Serial.print(" / ");
  //Serial.print(pitch);
  //Serial.print(" / ");

  Serial.print(Z[0]); Serial.print(" / ");
  Serial.print(Z[1]); Serial.print(" / ");
  Serial.print(Z[2]); Serial.print(" / ");
  Serial.println();
  //Serial.println(yaw)
  //Serial.println(" / ");
}

double Trans(struct Axis value) {
  return value.add + value.rest + (value.multiple * 255);
}
