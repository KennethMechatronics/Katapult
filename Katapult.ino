#include <Servo.h>
#include <LiquidCrystal.h>

const int step = 13;
const int dir = 12;
const int ms1 = 11;

const int echoPin = 9;
const int trigPin = 8;

const int knappLast = A3;
const int knappKast = A4;
const int sjokksensor = A5;

const int endebryter = A2;

const int servoPot = A0;
const int potKastStep = A1;

const float graderPrStep = 1.8;
float antStep;
int iAntStep;
int utgangsvinkel;
const int delayKast = 1000;  // 800 er en bra verdi.

bool harKastet = false;
bool traff = false;

unsigned long t0;
unsigned long t1;
unsigned long t2;
float distanse = 0;

const int timeout = 5000;

const float lengdeKasteArm = 0.147;  //[m]
const float pi = 3.1415;
float buelengde = 0.0;
const float mBadeand = 0.005;  // [kg]

unsigned long fortid;
int intervall = 500;

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
//                rs en d4 d5 d6 d7

byte katapult[] = {
  B00011,
  B00010,
  B00100,
  B01000,
  B10000,
  B11111,
  B11011,
  B11011
};

byte prosjektil[] = {
  B00000,
  B00110,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

Servo servo;
int servoPos;
const int servoNullpunkt = 100;

float tall[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const String dataNavn[9] = { "Akselerasjonstid", "Total tid", "Flytid", "Akselerasjon", "Kraft", "Utgangshastighet", "Horisontal hastighet", "Utgangshoyde", "Max banehoyde" };






void setup() {

  Serial.begin(9600);
  Serial.println("Programmet starter...");

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Katapult");
  lcd.createChar(0, katapult);
  lcd.createChar(1, prosjektil);
  lcd.setCursor(15, 1);
  lcd.write(byte(0));
  lcd.setCursor(13, 1);
  lcd.write(byte(1));



  for (int i = 11; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }

  servo.attach(10);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(knappLast, INPUT_PULLUP);
  pinMode(knappKast, INPUT_PULLUP);
  pinMode(endebryter, INPUT_PULLUP);
  pinMode(sjokksensor, INPUT_PULLUP);

  pinMode(servoPot, INPUT);
  pinMode(potKastStep, INPUT);

  digitalWrite(ms1, LOW);

  if (hentDistanse() <= 0.0) {
    Serial.println("Distanse kunne ikke hentes");
  }

  Serial.println("Programmet er startet!");
}

void loop() {
  unsigned long naatid = millis();

  int servoPos = map(analogRead(servoPot), 0, 1023, 0, 180);
  servo.write(servoPos);

  int utgangsvinkel = map(analogRead(potKastStep), 0, 1023, 0, 100);
  antStep = (utgangsvinkel / graderPrStep);
  iAntStep = antStep;

  float buelengde = 2.0 * pi * lengdeKasteArm * (utgangsvinkel / 360.0);  // [m]

  if (naatid - fortid >= intervall) {
    fortid = naatid;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Utg.Vinkel ");

    lcd.print(utgangsvinkel);

    lcd.setCursor(0, 1);
    lcd.print("Retning ");
    lcd.print(servoPos);
  }

  if (digitalRead(knappKast) == LOW) {
    distanse = hentDistanse();
    delay(50);
    Serial.println("Kaster");
    delay(50);
    kast();
    delay(50);
  }

  if (digitalRead(knappLast) == LOW) {
    lastProsjektil();
  }

  if (harKastet == true) {
    if (digitalRead(sjokksensor) == LOW) {
      t2 = millis();
      Serial.println("Treff");

      beregninger(t0, t1, t2, buelengde, distanse, utgangsvinkel, mBadeand, lengdeKasteArm, pi);

    } else if (digitalRead(sjokksensor) == HIGH && millis() >= t1 + timeout) {
      Serial.println("Du bomma");
      harKastet = !harKastet;
      delay(1000);
      lastProsjektil();
    }
  }
}


void kast() {
  digitalWrite(dir, LOW);
  digitalWrite(ms1, LOW);

  t0 = millis();

  for (int i = 0; i < iAntStep; i++) {
    digitalWrite(step, HIGH);
    delayMicroseconds(delayKast);
    digitalWrite(step, LOW);
    delayMicroseconds(delayKast);
  }

  t1 = millis();

  harKastet = true;
}

void lastProsjektil() {
  Serial.println("Kjorer til endeposisjon");  // Kunne ikke ha denne inne i for-loopen pga. treghet og støy.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Kjorer til ");
  lcd.setCursor(0, 1);
  lcd.print("endeposisjon ");

  servo.write(servoNullpunkt);
  delay(500);

  digitalWrite(dir, HIGH);  // Kjører stepperen bakover
  digitalWrite(ms1, HIGH);  // senker til HALF STEP

  while (digitalRead(endebryter) == HIGH) {
    for (int j = 0; j < 10; j++) {
      digitalWrite(step, HIGH);
      delayMicroseconds(1500);
      digitalWrite(step, LOW);
      delayMicroseconds(1500);
    }

    if (digitalRead(endebryter) == LOW) {
      Serial.println("Endebryter naad");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Endeposisjon");
      lcd.setCursor(0, 1);
      lcd.print("naad");
      delay(1000);
      return;
    }
  }
}

float hentDistanse() {

  unsigned int tid = 0;

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  tid = pulseIn(echoPin, HIGH);
  distanse = (tid * 0.343) / 2.0;
  distanse = distanse / 1000.0;

  return distanse;  // returnerer distanse [m]
}

float beregninger(long t0, long t1, long t2, float buelengde, float distanse, float utgangsvinkel, float mBadeand, float lengdeKasteArm, float pi) {
  Serial.println("Starter beregninger...");

  float aksTid = (t1 - t0) / 1000.0;                                                                             // [s]
  float totTid = (t2 - t0) / 1000.0;                                                                             // [s]
  float flytid = (t2 - t1) / 1000.0;                                                                             // [s]
  float akselerasjon = (2.0 * ((buelengde) - (aksTid))) / pow(aksTid, 2);                                        // [m/s^2]
  float Fkast = mBadeand * akselerasjon;                                                                         // [N]
  float utgangshastighet = akselerasjon * (aksTid);                                                              // [m/s]
  float horisontalHastighet = distanse / (flytid);                                                               // [m/s]
  float utgangshoyde = lengdeKasteArm * sin(utgangsvinkel * (pi / 180)) + 0.1;                                   // [m] over bakken    (0.1 er høyde til omdreiningspunkt)
  float maxBanehoyde = (pow(utgangshastighet, 2) * pow(sin(utgangsvinkel * (pi / 180.0)), 2.0)) / (2.0 * 9.81);  // [m]

  tall[0] = aksTid;
  tall[1] = totTid;
  tall[2] = flytid;
  tall[3] = akselerasjon;
  tall[4] = Fkast;
  tall[5] = utgangshastighet;
  tall[6] = horisontalHastighet;
  tall[7] = utgangshoyde;
  tall[8] = maxBanehoyde;

  for (int i = 0; i <= 8; i++) {
    Serial.print(dataNavn[i] + "\t" + tall[i]);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(dataNavn[i]);
    lcd.setCursor(0, 1);
    lcd.print(tall[i]);
    switch (i) {
      case 0 ... 2:
        Serial.print(" [s]");
        lcd.print("         [s]");
        break;
      case 3:
        Serial.print(" [m/s^2]");
        lcd.print("    [m/s^2]");
        break;
      case 4:
        Serial.print(" [N]");
        lcd.print("         [N]");
        break;
      case 5 ... 6:
        Serial.print(" [m/s]");
        lcd.print("       [m/s]");
        break;
      case 7 ... 8:
        Serial.print(" [m]");
        lcd.print("         [m]");
        break;
    }
    Serial.println();
    delay(3000);
  }
  lcd.clear();
  harKastet = !harKastet;
}





//
