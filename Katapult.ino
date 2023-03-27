/*
  Filnavn: Katapult.ino
  Versjon: V1.0
  Revisjon: V1.0 -29.03.23 - Første versjon - Kenneth Paulsen

  Program: Programmet er laget for å kaste en badeand fra Privat Bar på Kongsberg. Det mekaniske designet er slik at konstruksjonen er en elektrisk katapult.
  Katapulten styres ved to potmetre som stiller inn retning (servomotor) og utgangsvinkel på badeanden, altså hvor mange grader steppermotoren går før den 
  bråstopper. For å finne nullpunkt på steppermotoren kjøres katapultarmen til en endebryteren fra den gamle stekeovnen min. 

  Det er koblet en sjokksensor med lang ledning til Arduinoen. Om denne sjokksensoren registrerer rørelse har badeanden truffet målet! 
  Om så skjer starter en funksjon jeg har kalt "beregninger". Denne får noe data fra selve kastet og beregner teoretisk data angående badeandens 
  bane, flyvetid osv. De beregnede verdiene sendes til seriell monitor og en LCD-skjerm. 

  Datablad og info: 
  Sjokksensor:      https://www.velleman.eu/downloads/29/vma312_a4v01.pdf
  Stepperdriver:    https://learn.sparkfun.com/tutorials/big-easy-driver-hookup-guide?_ga=2.226144732.75772450.1679860848-749363297.1654960242 
  Stepperdriver IC: https://cdn.sparkfun.com/datasheets/Robotics/A4988-Datasheet.pdf
  Steppermotor:     https://www.elfadistrelec.no/Web/Downloads/_t/ds/SF2421-10B41_eng_tds.pdf
  Servomotor:       http://myosuploads3.banggood.com/products/20220907/20220907032153DS3225datasheet.pdf
  Avstandssensor:   https://datasheetspdf.com/pdf-file/813041/ETC/HY-SRF05/1
  5V:               https://www.elfadistrelec.no/Web/Downloads/xc/_e/teL78xx-L78xxC_e.pdf
  6V:               https://4donline.ihs.com/images/VipMasterIC/IC/SGST/SGST-S-A0006334694/SGST-S-A0006334694-1.pdf?hkey=52A5661711E402568146F3353EA87419
  24V:              https://www.kjell.com/globalassets/mediaassets/822048_45198_manual_en_no_sv.pdf?ref=F6866994D3
  Kondensatorer:    Ukjent sett fra Kina
  Micro/endebryter: Umerket fra Bosch stekeovn
  Avstandssensor:   Umerket avart av HC-SR04 eller lignende https://docs.rs-online.com/8bc5/A700000007388293.pdf
  Konstruksjon:     3D-printet med PLA.

*/

#include <Servo.h>                                          // Inkluderer biblioteket Servo.h
#include <LiquidCrystal.h>                                  // Inkluderer biblioteket LiquidCrystal.h

//                rs en d4 d5 d6 d7
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);                        // Setter opp LCD-skjermen 

Servo servo;

const int trigPin = 8;
const int step = 10;
const int ms1 = 11;
const int dir = 12;

const int sjokksensor = 13;
const int knappLast = A0;   // 14
const int knappKast = A1;   // 15
const int endebryter = A2;  // 16

const int potKastStep = A3;  // 17
const int servoPot = A4;     // 18
const int echoPin = A5;      // 19

int servoPos;
const int servoNullpunkt = 100;


float antStep;
int iAntStep;
int utgangsvinkel;
float buelengde;  // [m]
const float graderPrStep = 1.8;
const float pi = 3.1415;
const float lengdeKasteArm = 0.150;  // [m]
const int delayKast = 900;

float distanse = 0.0;

unsigned long t0;
unsigned long t1;
unsigned long t2;

unsigned long fortid;
const int intervall = 500;
const int timeout = 5000;

bool harKastet = false;
bool traff = false;

byte katapult[] = {  // https://maxpromer.github.io/LCD-Character-Creator/
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

  for (int i = 8; i <= 12; i++) {
    pinMode(i, OUTPUT);
  }

  servo.attach(9);

  for (int j = 13; j <= 16; j++) {
    pinMode(j, INPUT_PULLUP);
  }

  for (int k = 17; k <= 19; k++) {
    pinMode(k, INPUT);
  }

  digitalWrite(ms1, LOW);

  if (hentDistanse() <= 0.0) {
    Serial.println("Distanse kunne ikke hentes");
  }

  Serial.println("Programmet er startet!");
}

void loop() {
  unsigned long naatid = millis();



  int utgangsvinkel = map(analogRead(potKastStep), 0, 1023, 0, 100);
  antStep = (utgangsvinkel / graderPrStep);
  iAntStep = antStep;

  if (harKastet == true) {
    if (digitalRead(sjokksensor) == LOW) {
      t2 = millis();
      Serial.println("Treff");

      float buelengde = 2.0 * pi * lengdeKasteArm * (utgangsvinkel / 360.0);  // [m]

      beregninger(t0, t1, t2, buelengde, distanse, utgangsvinkel, lengdeKasteArm, pi);

    } else if (digitalRead(sjokksensor) == HIGH && millis() >= t1 + timeout) {
      Serial.println("Du bomma");
      harKastet = !harKastet;
      delay(1000);
      lastProsjektil();
    }
  }

  int servoPos = map(analogRead(servoPot), 0, 1023, 0, 180);
  servo.write(servoPos);

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
  }

  if (digitalRead(knappLast) == LOW) {
    lastProsjektil();
  }
}


void kast() {

  digitalWrite(dir, LOW);
  digitalWrite(ms1, LOW);
  Serial.println(String(iAntStep) + " step");

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
    for (int l = 0; l < 10; l++) {
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
      delay(700);
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


void beregninger(long t0, long t1, long t2, float buelengde, float distanse, int utgangsvinkel, float lengdeKasteArm, float pi) {
  const float mBadeand = 5.0;  // [g]

  Serial.println("Starter beregninger...");
  Serial.println("Data inn til funksjonen: \n");
  Serial.println("t0         \t" + String(t0) + " [ms]");
  Serial.println("t1         \t" + String(t1) + " [ms]");
  Serial.println("t2         \t" + String(t2) + " [ms]");
  Serial.println("Buelengde  \t" + String(buelengde) + " [m]");
  Serial.println("Distanse   \t" + String(distanse) + " [m]");
  Serial.println("Utg.vink   \t" + String(utgangsvinkel) + "   [*]");
  Serial.println("mBadeand   \t" + String(mBadeand) + " [g]");
  Serial.println("lengdeKArm \t" + String(lengdeKasteArm) + " [m]");
  Serial.println("pi         \t" + String(pi) + "\n");



  float tall[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  const String dataNavn[9] = { 
    "Akselerasjonstid    "
  , "Total tid           "
  , "Flytid              "
  , "Akselerasjon        "
  , "Kraft               "
  , "Utgangshastighet    "
  , "Horisontal hastighet"
  , "Utgangshoyde        "
  , "Max banehoyde       " };

  float aksTid = (t1 - t0) / 1000.0;                                                                             // [s]
  float totTid = (t2 - t0) / 1000.0;                                                                             // [s]
  float flytid = (t2 - t1) / 1000.0;                                                                             // [s]
  float vinkelRadiusRad = utgangsvinkel * (pi/180);                                                              // [rad]
  float vinkelhastighet = vinkelRadiusRad / aksTid;                                                              // [rad/s]
  float vinkelakselerasjon = vinkelhastighet / aksTid;                                                           // [rad/s^2]
  float akselerasjon = lengdeKasteArm * vinkelakselerasjon;                                                      // [m/s^2]   Vinkelakselerasjon
  //float akselerasjonRettlinjet = (2.0 * (buelengde - aksTid)) / pow(aksTid, 2);                                // [m/s^2]   Rettlinjet akselerasjon, som blir feil å bruke
  float Fkast = (mBadeand / 1000) * akselerasjon;                                                                // [N]
  float utgangshastighet = akselerasjon * (aksTid);                                                              // [m/s]
  float horisontalHastighet = distanse / (flytid);                                                               // [m/s]
  float utgangshoyde = lengdeKasteArm * sin(utgangsvinkel * (pi / 180)) + 0.1;                                   // [m] over bakken    (0.1 er høyde til omdreiningspunkt)
  float maxBanehoyde = (pow(utgangshastighet, 2) * pow(sin(utgangsvinkel * (pi / 180.0)), 2.0)) / (2.0 * 9.81);  // [m]

  // Serial.println(akselerasjonRettlinjet);
  tall[0] = aksTid;
  tall[1] = totTid;
  tall[2] = flytid;
  tall[3] = akselerasjon;
  tall[4] = Fkast;
  tall[5] = utgangshastighet;
  tall[6] = horisontalHastighet;
  tall[7] = utgangshoyde;
  tall[8] = maxBanehoyde;

  for (int m = 0; m <= 8; m++) {
    Serial.print(dataNavn[m] + "  " + tall[m]);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(dataNavn[m]);
    lcd.setCursor(0, 1);
    lcd.print(tall[m]);
    switch (m) {
      case 0 ... 2:
        Serial.print(" [s]");
        lcd.setCursor(13, 1);
        lcd.print("[s]");
        break;
      case 3:
        Serial.print("[m/s^2]");
        lcd.setCursor(9, 1);
        lcd.print("[m/s^2]");
        break;
      case 4:
        Serial.print(" [N]");
        lcd.setCursor(13, 1);
        lcd.print("[N]");
        break;
      case 5 ... 6:
        Serial.print(" [m/s]");
        lcd.setCursor(11, 1);
        lcd.print("[m/s]");
        break;
      case 7 ... 8:
        Serial.print(" [m]");
        lcd.setCursor(13, 1);
        lcd.print("[m]");
        break;
    }
    Serial.println();
    delay(2000);
  }
  lcd.clear();

  harKastet = !harKastet;
}





//
