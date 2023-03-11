#include <Servo.h>

const int kastStep = 13;
const int kastDir = 12;
const int ms1 = 11;
const int laser = 10;

const int sjokksensor = 8;
const int knappKast = 7;
const int knappOK = 6;
const int knappVerdiOpp = 5;
const int knappVerdiNed = 4;

const int pot1 = A0;
const int pot2 = A1;
const int joystickX = A2;
const int joystickY = A3;

Servo servo;
int servoNullpunkt = 20;
int servoPos;

int stepTid = 500;  //mikrosekund


void setup() {
  for (int i = 10; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }

  servo.attach(9);

  for (int j = 4; j <= 8; j++) {
    pinMode(j, INPUT_PULLUP);
  }

  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);
  pinMode(joystickY, INPUT);
  pinMode(joystickX, INPUT);



  Serial.begin(9600);

  servo.write(servoNullpunkt);
  digitalWrite(ms1, LOW);
}

void loop() {
  int servoPos = map(analogRead(joystickX), 0, 1023, 0, 180);
  Serial.println(servoPos);
  servo.write(servoPos);








  delay(0);
}








//
