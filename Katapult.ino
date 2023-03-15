#include <Servo.h>
#include <Wire.h>

const int step = 13;
const int dir = 12;
const int ms1 = 11;

const int knappKast = 7;
const int knappLast = 5;

const int endebryter = 3;

const int servoPot = A0;
const int potKastStep = A1;

const float graderPrStep = 1.8;
float fKastGrader;

Servo servo;
int servoPos;
const int servoNullpunkt = 100;

int slaveAdresse = 2;

int data[] = {};
int dataArrayStorrelse = 2;
 

unsigned long fortid;
const int intervallDatasending = 700;



void setup() {

  Serial.begin(9600);
  Serial.println("Programmet starter...");

  Wire.begin();


  for (int i = 10; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }

  servo.attach(9);

  for (int j = 3; j <= 8; j++) {
    pinMode(j, INPUT_PULLUP);
  }

  pinMode(servoPot, INPUT);
  pinMode(potKastStep, INPUT);

  digitalWrite(ms1, LOW);

  Serial.println("Programmet er startet!");
}

void loop() {
  unsigned long naatid = millis();

  int servoPos = map(analogRead(servoPot), 0, 1023, 0, 180);
  servo.write(servoPos);

  int antStepKast = map(analogRead(potKastStep), 0, 1023, 100, 0);
  float fKastGrader = antStepKast * graderPrStep;

  //Serial.println(data[1]);

  if (naatid - fortid >= intervallDatasending) {
    fortid = naatid;

    byte bKastGrader = fKastGrader;
    byte bServoPos = servoPos;
    data[0] = bServoPos;
    data[1] = bKastGrader;

    Wire.beginTransmission(slaveAdresse);
    Wire.write(data[1]);
    Wire.endTransmission();
    Serial.println("Sender bServoPos til slave");
  }







  if (digitalRead(knappKast) == LOW) {
    kast();
  }

  if (digitalRead(knappLast) == LOW) {
    lastProsjektil();
  }
}

void kast() {
  Serial.println("Kaster");
  digitalWrite(dir, LOW);
  digitalWrite(ms1, LOW);

  int antStepKast = map(analogRead(potKastStep), 0, 1023, 100, 0);

  for (int i = 0; i < step; i++) {

    digitalWrite(step, HIGH);
    delayMicroseconds(500);
    digitalWrite(step, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}

void lastProsjektil() {
  servo.write(servoNullpunkt);
  delay(1000);

  digitalWrite(dir, HIGH);  // Kjører stepperen bakover
  digitalWrite(ms1, HIGH);  // senker til HALF STEP

  while (digitalRead(endebryter) == HIGH) {
    for (int j = 0; j < 10; j++) {
      digitalWrite(step, HIGH);
      delayMicroseconds(1200);
      digitalWrite(step, LOW);
      delayMicroseconds(1200);
    }
    Serial.println("Kjorer til endeposisjon");
    if (digitalRead(endebryter) == LOW) {
      Serial.println("Endebryter naad");
      bool endebryterNaad = true;
      delay(1000);

      if (endebryterNaad == true) {
        //return;
        digitalWrite(dir, LOW);
        digitalWrite(ms1, HIGH);

        for (int i = 0; i < 30; i++) {  // Kjører litt opp etter endebryteren er nådd.
          digitalWrite(step, HIGH);
          delayMicroseconds(1000);
          digitalWrite(step, LOW);
          delayMicroseconds(1000);
        }
        Serial.println("kjorer litt opp");  // Kunne ikke ha denne i for-loopen fordi Arduinoen er for treg (skapte ulyd og lav fart)
      }
      endebryterNaad = !endebryterNaad;
      delay(1000);
      return;  // er denne nødvendig?
    }
  }
}






//
