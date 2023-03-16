#include <Servo.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
//                rs en d4 d5 d6 d7

Servo servo;
int servoPos;
const int servoNullpunkt = 100;

const int step = 13;
const int dir = 12;
const int ms1 = 11;

const int knappLast = 9;
const int knappKast = 8;

const int endebryter = A2;

const int servoPot = A0;
const int potKastStep = A1;

const float graderPrStep = 1.8;
float fKastGrader;

unsigned long fortid;
int intervall = 500;


void setup() {

  Serial.begin(9600);
  Serial.println("Programmet starter...");

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Katapult");


  for (int i = 11; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }

  servo.attach(10);

  for (int j = 8; j <= 9; j++) {
    pinMode(j, INPUT_PULLUP);
  }
  pinMode(endebryter, INPUT_PULLUP);

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



  if (naatid - fortid >= intervall) {
    fortid = naatid;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Utg.Vinkel ");
    int iKastGrader = fKastGrader;
    lcd.print(iKastGrader);

    lcd.setCursor(0, 1);
    lcd.print("Retning ");
    lcd.print(servoPos);
  }


  if (digitalRead(knappKast) == LOW) {
    kast();
    delay(100);
    Serial.println("Kaster");
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

  for (int i = 0; i < antStepKast; i++) {

    digitalWrite(step, HIGH);
    delayMicroseconds(800);
    digitalWrite(step, LOW);
    delayMicroseconds(800);
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
      delayMicroseconds(1500);
      digitalWrite(step, LOW);
      delayMicroseconds(1500);
    }
    Serial.println("Kjorer til endeposisjon");
    if (digitalRead(endebryter) == LOW) {
      Serial.println("Endebryter naad");
      bool endebryterNaad = true;
      delay(1000);

      if (endebryterNaad == true) {
        endebryterNaad = !endebryterNaad;
        delay(200);
        return;
      }
    }
  }
}






//
