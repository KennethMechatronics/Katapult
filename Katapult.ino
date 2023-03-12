#include <Servo.h>

const int kastStep = 13;
const int kastDir = 12;
const int ms1 = 11;
const int laser = 10;

const int sjokksensor = 8;
const int knappKast = 7;
//const int knappOK = 6;
const int knappLast = 5;
//const int knappVerdiNed = 4;
const int endebryter = 3;

const int servoPot = A0;
const int potutkastStep = A1;

const float graderPrStep = 1.8;



Servo servo;
int servoPos;


// int stepTid = 500;  //mikrosekund


void setup() {
  for (int i = 10; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }

  servo.attach(9);

  for (int j = 3; j <= 8; j++) {
    pinMode(j, INPUT_PULLUP);
  }

  pinMode(servoPot, INPUT);
  pinMode(potutkastStep, INPUT);




  Serial.begin(9600);


  digitalWrite(ms1, LOW);
}

void loop() {
  int servoPos = map(analogRead(servoPot), 0, 1023, 0, 180);
  servo.write(servoPos);
  //Serial.println("loop");

  int utkastStep = map(analogRead(potutkastStep), 0, 1023, 100, 0);
  //Serial.println(utkastStep);                           
  float utkastGrader = utkastStep * graderPrStep;
  Serial.println(utkastGrader); // Kan denne sendes til arduino 2?
  

  if (digitalRead(knappKast) == LOW) {
    kast();
  }

  if (digitalRead(knappLast) == LOW) {
    lastProsjektil();
  }
}

void kast() {
  Serial.println("Kaster");
  digitalWrite(kastDir, LOW);
  digitalWrite(ms1, LOW);
  
  int utkastStep = map(analogRead(potutkastStep), 0, 1023, 100, 0);
 
  for (int i = 0; i < utkastStep; i++) {

    digitalWrite(kastStep, HIGH);
    delayMicroseconds(500);
    digitalWrite(kastStep, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}

void lastProsjektil() {
  servo.write(100);
  delay(1000);

  digitalWrite(kastDir, HIGH);  // Kjører stepperen bakover
  digitalWrite(ms1, HIGH);      // senker til HALF STEP

  while (digitalRead(endebryter) == HIGH) {
    for (int j = 0; j < 10; j++) {
      digitalWrite(kastStep, HIGH);
      delayMicroseconds(1200);
      digitalWrite(kastStep, LOW);
      delayMicroseconds(1200);
    }
    Serial.println("Kjorer til endeposisjon");
    if (digitalRead(endebryter) == LOW) {
      Serial.println("Endebryter naad");
      bool endebryterNaad = true;
      delay(1000);

      if (endebryterNaad == true) {
        digitalWrite(kastDir, LOW);
        digitalWrite(ms1, HIGH);

        for (int i = 0; i < 30; i++) {
          digitalWrite(kastStep, HIGH);
          delayMicroseconds(1000);
          digitalWrite(kastStep, LOW);
          delayMicroseconds(1000);
        }
        Serial.println("kjorer litt opp");      // Kunne ikke ha denne i for-loopen fordi Arduinoen er for treg (skapte ulyd og lav fart)
      }
      endebryterNaad = !endebryterNaad;
      delay(1000);
      return;
    }
  }
}






//
