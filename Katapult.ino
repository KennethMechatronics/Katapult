/*
  Filnavn: Katapult.ino
  Versjon: V1.0
  Revisjon: V1.0 - 28.03.23 - Første versjon - Kenneth Paulsen FTMEN3

  Program: Programmet er laget for å katapultere en badeand fra Privat Bar på Kongsberg. Det mekaniske designet er slik at konstruksjonen ligner en 
  elektrisk katapult.
  Katapulten styres ved to potmetre som stiller inn retning (servomotor) og utgangsvinkel på badeanden, altså hvor mange grader steppermotoren går før den 
  bråstopper. For å finne nullpunkt på steppermotoren kjøres katapultarmen til en endebryteren fra den gamle stekeovnen min. 

  Det er koblet en sjokksensor med lang ledning til Arduinoen. Om denne sjokksensoren registrerer rørelse om badeanden treffer målet! 
  Om så skjer starter en funksjon jeg har kalt "beregninger". Denne får noe data fra selve kastet og beregner teoretisk data angående badeandens 
  bane, flyvetid osv. De beregnede verdiene sendes til seriell monitor i fint leselig format og en LCD-skjerm. 
 
  DISCLAIMER: 
  Angående beregningene så er luftmotstand ikke tatt i betraktning og akselerasjonen beregnes/måles på feil måte. 
  Det hadde vært bedre å måle akselerasjonen med et potmeter eller lignenede. Slik den beregnes nå sier jeg at den akselererer gjennom hele kastet mens den i virkeligheten akselererer 
  i en- kanskje to step før den har tilnærmet konstant fart til den når enden av For-loopen.
  Til sammenligning kan en Tesla Model S Plaid akselerere fra 0 til 100 på 2,07s. ~> 48,3 m/s^2


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

#include <Servo.h>                                    // Inkluderer biblioteket Servo.h
#include <LiquidCrystal.h>                            // Inkluderer biblioteket LiquidCrystal.h

//                rs en d4 d5 d6 d7
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);                  // Setter opp LCD-skjermens pinner

Servo servo;                                          // Lager et servoobjekt kalt servo
int servoPos;                                         // Variabel for å holde servopossisjon. 
const int servoNullpunkt = 100;                       // Konstant variabel.  Servoens nullpunkt = 100

const int trigPin = 8;                                // Konstant variabel. Definerer pinne 8 som trigPin
const int step = 10;                                  // Konstant variabel.  Definerer pinne 10 som step
const int ms1 = 11;                                   // Konstant variabel.  Definerer pinne 11 som ms1
const int dir = 12;                                   // Konstant variabel.  Definerer pinne 12 som dir

const int sjokksensor = 13;                           // Konstant variabel.  Definerer pinne 13 som sjokksensor
const int knappLast = A0;   // 14                     // Konstant variabel.  Definerer pinne A0 / 14 som knappLast
const int knappKast = A1;   // 15                     // Konstant variabel.  Definerer pinne A1 / 15 som knappKast
const int endebryter = A2;  // 16                     // Konstant variabel.  Definerer pinne A2 / 16 som endebryter

const int potKastStep = A3;  // 17                    // Konstant variabel.  Definerer pinne A3 / 17 som potKastStep
const int servoPot = A4;     // 18                    // Konstant variabel.  Definerer pinne A4 / 18 som servoPot
const int echoPin = A5;      // 19                    // Konstant variabel.  Definerer pinne A5 / 19 som echoPin

float antStep;                                        // Variabel i kalkulasjon av antall step vs grader
int iAntStep;                                         // int-versjon av varabelen slik at den kan brukes i en for-loop
int utgangsvinkel;                                    // Variabel i kalkulasjon av utgangsvinkel 
const float graderPrStep = 1.8;                       // Konstant variabel.  Variabel brukt i kalkulasjon for å finne utgangsvinkel vs step
const int delayKast = 900;                            // Konstant variabel.  Varighet på delay mellom veksling høy lav på steppermotoren

float distanse = 0.0;                                 // Variabel for å lagre målt distanse

unsigned long t0;                                     // Variabel for å holde på tid
unsigned long t1;                                     // Variabel for å holde på tid
unsigned long t2;                                     // Variabel for å holde på tid
unsigned long fortid;                                 // Variabel for å holde på tid
const int intervall = 500;                            // Konstant variabel.  Intervall
const int timeout = 5000;                             // Konstant variabel.  timeout-tid

bool harKastet = false;                               // bool som settes om man har kastet eller ikke
bool traff = false;                                   // bool som settes om man treffer målet/sjokksensoren eller ikke (timeout)

byte katapult[] = {                                   // Egendefinert karakter på LCD-skjermen som et array 
  B00011,                                             // Noen ruter blanke andre hvite
  B00010,                                             // Noen ruter blanke andre hvite
  B00100,                                             // Noen ruter blanke andre hvite
  B01000,                                             // Noen ruter blanke andre hvite
  B10000,                                             // Noen ruter blanke andre hvite
  B11111,                                             // Noen ruter blanke andre hvite
  B11011,                                             // Noen ruter blanke andre hvite
  B11011                                              // Noen ruter blanke andre hvite
};                                                    // Array slutt. Generert på: https://maxpromer.github.io/LCD-Character-Creator/

byte prosjektil[] = {                                 // Egendefinert karakter på LCD-skjermen som et array
  B00000,                                             // Noen ruter blanke andre hvite
  B00110,                                             // Noen ruter blanke andre hvite
  B00110,                                             // Noen ruter blanke andre hvite
  B00000,                                             // Alle ruter blanke
  B00000,                                             // Alle ruter blanke
  B00000,                                             // Alle ruter blanke
  B00000,                                             // Alle ruter blanke
  B00000                                              // Alle ruter blanke
};                                                    // Array slutt. Generert på: https://maxpromer.github.io/LCD-Character-Creator/

void setup() {                                              // Setup starter, kode som kjøres en gang

  Serial.begin(9600);                                       // Starter seriell monitor, hastighet: 9600
  Serial.println("Programmet starter...");                  // Printer tekst + linjeskift i seriell monitor

  lcd.begin(16, 2);                                         // Starter LCD-skjermen med parametrene 16x2
  lcd.clear();                                              // Sletter alt som var på LCD-skjermen
  lcd.setCursor(4, 0);                                      // Setter cursoren i LCD-skjermen på plass x, linje y
  lcd.print("Katapult");                                    // Printer tekst  i seriell monitor
  lcd.createChar(0, katapult);                              // Lager karakteren ut i fra det første arrayet (katapult[])
  lcd.createChar(1, prosjektil);                            // lager karakteren ut i fra det andre arrayet (prosjektil[])
  lcd.setCursor(15, 1);                                     // Setter cursoren i LCD-skjermen på plass x, linje y
  lcd.write(byte(0));                                       // Printert det første arrayet
  lcd.setCursor(13, 1);                                     // Setter cursoren i LCD-skjermen på plass x, linje y
  lcd.write(byte(1));                                       // Printer det andre arrayet

  for (int i = 8; i <= 12; i++) {                           // For-loop med tellevariabel i
    pinMode(i, OUTPUT);                                     // Setter tallet tellevariabelen er lik til OUTPUT
  }                                                         // For-loop SLUTT

  servo.attach(9);                                          // Fester/setter servo(objektet) til pinne 9

  for (int j = 13; j <= 16; j++) {                          // For-loop med tellevariabel j
    pinMode(j, INPUT_PULLUP);                               // Setter tallet tellevariabelen er lik til INPUT_PULLUP
  }                                                         // For-loop SLUTT

  for (int k = 17; k <= 19; k++) {                          // For-loop med tellevariabel k
    pinMode(k, INPUT);                                      // Setter tallet tellevariabelen er lik til INPUT
  }                                                         // For-loop SLUTT

  digitalWrite(ms1, LOW);                                   // Setter ms1 på stepperdriveren LOW. Slik at man får FULL STEP på motoren

  if (hentDistanse() <= 0.0) {                              // hvis den returnerte vedien fra funksjonen er mindre enn 0.0, gjøres følgende.
    Serial.println("Distanse kunne ikke hentes");           // Printer tekst + linjeskift i seriell monitor
  }                                                         // if SLUTT

  Serial.println("Programmet er startet!");                 // Printer tekst + linjeskift i seriell monitor
}                                                           // Void Setup SLUTT

void loop() {                                                                           // Void loop starter, dette kjøres hele tiden, untatt om man er i en annen funksjon
  unsigned long naatid = millis();                                                      // Lagrer verdien fra millis() i variabelen naatid 

  int utgangsvinkel = map(analogRead(potKastStep), 0, 1023, 0, 100);                    // Leser potmetret og mapper om til 0 -> 100 og lagrer i variabelen utgangsvinkel 
  antStep = (utgangsvinkel / graderPrStep);                                             // Regner ut hvor mange step den innstilte vinkelen tilsvarer       
  iAntStep = antStep;                                                                   // Gjør om antStep til iAntStep. Fordi antStep blir en float etter å ha delt på graderPrStep. For-looper godtar ikke float

  int servoPos = map(analogRead(servoPot), 0, 1023, 0, 180);                            // Leser potmetret og mapper om til 0 -> 180 og lagrer i variabelen servoPos
  servo.write(servoPos);                                                                // Skriver verdien fra servoPos til servoen

  if (naatid - fortid >= intervall) {                                                   // Hvis verdien i naatid - fortid er større eller lik intervall gjøres følgende
    fortid = naatid;                                                                    // setter fortid lik naatid
    lcd.clear();                                                                        // Sletter alt som var på LCD-skjermen
    lcd.setCursor(0, 0);                                                                // Setter cursoren i LCD-skjermen på plass x, linje y
    lcd.print("Utg.Vinkel ");                                                           // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor

    lcd.print(utgangsvinkel);                                                           // Printer verdien i utgangsvinkel på LCD-skjermen

    lcd.setCursor(0, 1);                                                                // Setter cursoren i LCD-skjermen på plass x, linje y
    lcd.print("Retning ");                                                              // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor
    lcd.print(servoPos);                                                                // Printer verdien i servoPos på LCD-skjermen
  }                                                                                     // if SLUTT

  if (digitalRead(knappLast) == LOW) {                                                  // Hvis knappLast blir lav gjøres følgende
    lastProsjektil();                                                                   // Starter funksjonen lastProsjektil()
  }                                                                                     // if SLUTT

  if (digitalRead(knappKast) == LOW) {                                                  // Hvis knappKast blir lav gjøres følgende
    distanse = hentDistanse();                                                          // Lagrer den returnerte verdien fra hentDistanse() i distanse
    delay(50);                                                                          // Venter
    Serial.println("Kaster");                                                           // Printer tekst + linjeskift i seriell monitor
    delay(50);                                                                          // Venter
    kast();                                                                             // Starter funksjonen kast()
  }                                                                                     // if SLUTT

  if (harKastet == true) {                                                              // Hvis variabelen harKastet er sann gjøres følgende 
    if (digitalRead(sjokksensor) == LOW) {                                              // Hvis sjokksensor er lav(badeanda traff) gjøres følgende
      t2 = millis();                                                                    // lagrer den returnerte verdien fra millis() i t2
      Serial.println("Treff");                                                          // Printer tekst + linjeskift i seriell monitor

      beregninger(t0, t1, t2, distanse, utgangsvinkel);// Starter funksjonen beregninger og sender med en hel rekke variabler

    } else if (digitalRead(sjokksensor) == HIGH && millis() >= t1 + timeout) {          // Hvis sjokksensoren er høy OG den returnerte verdien fra millis() er større enn (t1 + variabelen timeout)
      Serial.println("Du bomma");                                                       // Printer tekst + linjeskift i seriell monitor
      harKastet = !harKastet;                                                           // Inverterer variablen 1->0 eller 0->1
      delay(1000);                                                                      // Venter
      lastProsjektil();                                                                 // Starter funksjonen lastProsjektil()
    }                                                                                   // else if SLUTT
  }                                                                                     // if SLUTT
}                                                                                       // Void loop SLUTT

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////EGNE FUNKSJONER///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void kast() {                                                // Funksjonen kast(), kaster badeanden og oppdaterer t1 og t2

  digitalWrite(dir, LOW);                                    // Setter retning på steppermotoren
  digitalWrite(ms1, LOW);                                    // Setter ms1 lav slik at man får FULL STEP
  Serial.println(String(iAntStep) + " step");                // Printer en omgjort iAntStep til String + tekst + linjeskift 

  t0 = millis();                                             // Setter t0 til den returnerte verdien fra millis()

  for (int l = 0; l <= iAntStep; l++) {                      // For-loop med tellevariabel l. Kjører så lenge l er mindre eller lik variabelen iAntStep
    digitalWrite(step, HIGH);                                // Sender høy puls til stepperdirveren
    delayMicroseconds(delayKast);                            // Venter
    digitalWrite(step, LOW);                                 // Sender høy puls til stepperdirveren
    delayMicroseconds(delayKast);                            // Venter
  }                                                          // For-loop SLUTT

  t1 = millis();                                             // Setter t1 til den returnerte verdien fra millis()

  harKastet = true;                                          // Setter variabelen harKastet til sann
}                                                            // kast() SLUTT


void lastProsjektil() {                                             // Funksjonen lastProsjektil() skal kjøre servoen og steppermotoren til endebryteren, slik at utgngspunktet for hvert kast blir likt 
  Serial.println("Kjorer til endeposisjon");                        // Printer tekst + linjeskift.  Kunne ikke ha denne inne i for-loopen pga. treghet og støy.
  lcd.clear();                                                      // Sletter alt som var på LCD-skjermen
  lcd.setCursor(0, 0);                                              // Setter cursoren i LCD-skjermen på plass x, linje y
  lcd.print("Kjorer til ");                                         // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor
  lcd.setCursor(0, 1);                                              // Setter cursoren i LCD-skjermen på plass x, linje y
  lcd.print("endeposisjon ");                                       // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor

  servo.write(servoNullpunkt);                                      // Skriver servoNullPunkt til servoen
  delay(500);                                                       // Venter

  digitalWrite(dir, HIGH);                                          // Kjører stepperen bakover
  digitalWrite(ms1, HIGH);                                          // Senker til HALF STEP. Det er nå 400 step pr. runde, istedet for 200

  while (digitalRead(endebryter) == HIGH) {                         // Så lenge endebryteren er høy gjøres følgende
    for (int m = 0; m < 10; m++) {                                  // For-loop med tellevariabel m. 10 var en fin verdi som ikke ga støy eller for mye bevegelse
      digitalWrite(step, HIGH);                                     // Sender høy puls til stepperdriveren 
      delayMicroseconds(1500);                                      // Venter
      digitalWrite(step, LOW);                                      // Sender lav puls til stepperdriveren
      delayMicroseconds(1500);                                      // Venter
    }                                                               // For-loop SLUTT

    if (digitalRead(endebryter) == LOW) {                           // Hvis endebryteren bli lav gjøres følgende
      Serial.println("Endebryter naad");                            // Printer tekst + linjeskift i seriell monitor
      lcd.clear();                                                  // Sletter alt som var på LCD-skjermen
      lcd.setCursor(0, 0);                                          // Setter cursoren i LCD-skjermen på plass x, linje y
      lcd.print("Endeposisjon");                                    // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor
      lcd.setCursor(0, 1);                                          // Setter cursoren i LCD-skjermen på plass x, linje y
      lcd.print("naad");                                            // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor
      delay(700);                                                   // Venter
      return;                                                       // Går ut av funksjonen
    }                                                               // if SLUTT
  }                                                                 // while SLUTT
}                                                                   // lastProsjektil() SLUTT


float hentDistanse() {                                        // Funksjonen hentDistanse, skal måle distansen til målet og returnere distansen til hvem-enn som spør 

  unsigned int tid = 0;                                       // Variabel som holder på tiden

  digitalWrite(trigPin, HIGH);                                // Setter trigPin høy
  delayMicroseconds(10);                                      // Venter 10 mikrosekunder
  digitalWrite(trigPin, LOW);                                 // Setter trigPin lav

  tid = pulseIn(echoPin, HIGH);                               // Lagrer tiden fra pulseIn på echoPin i variabelen tid
  distanse = (tid * 0.343) / 2.0;                             // Regner om tid * lydhastighet / 2
  distanse = distanse / 1000.0;                               // Regner om til meter

  return distanse;                                            // returnerer distanse [m]                      
}                                                             // float hentDistanse() SLUTT


void beregninger(long t0, long t1, long t2, float distanse, int utgangsvinkel) { // Funksjonen som beregner og viser data i monitor og på LCD. Alle data ut blir teoretiske da jeg ikke har noe feedback. Luftmotstand på badeanda er heller ikke medregnet
  Serial.println("Starter beregninger...");                                      // Printer tekst + linjeskift i seriell monitor
  
  const float mBadeand = 5.0;  // [g]                                           // Konstant variabel som holder badeandens vekt i gram
  const float pi = 3.1415;                                                      // Konstant variabel.  pi
  const float lengdeKatapultarm = 0.150;  // [m]                                // Konstant variabel.  Lengden på katapultarmen 

  float tall[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};               // float Array med 9 plasser som skal holde på de beregnede verdiene
  const String dataNavn[9] = {                                                  // Konstant String Array som holder tekst
    "Akselerasjonstid    "                                                      // plass 0
  , "Total tid           "                                                      // plass 1
  , "Flytid              "                                                      // plass 2
  , "Akselerasjon        "                                                      // plass 3
  , "Kraft               "                                                      // plass 4
  , "Utgangshastighet    "                                                      // plass 5
  , "Horisontal hastighet"                                                      // plass 6
  , "Utgangshoyde        "                                                      // plass 7
  , "Max banehoyde       " };                                                   // plass 8


  Serial.println("Data inn til funksjonen: \n");                                // Printer tekst + linjeskift i seriell monitor + ende et linjeskift

  Serial.println("t0         \t" + String(t0) + " [ms]");                       // Pinter tekst, TAB, verdien omgjort til en String, tekst + linjeskift
  Serial.println("t1         \t" + String(t1) + " [ms]");                       // Pinter tekst, TAB, verdien omgjort til en String, tekst + linjeskift
  Serial.println("t2         \t" + String(t2) + " [ms]");                       // Pinter tekst, TAB, verdien omgjort til en String, tekst + linjeskift
  Serial.println("Distanse   \t" + String(distanse) + " [m]");                  // Pinter tekst, TAB, verdien omgjort til en String, tekst + linjeskift
  Serial.println("Utg.vink   \t" + String(utgangsvinkel) + "   [*]");           // Pinter tekst, TAB, verdien omgjort til en String, tekst + linjeskift
  Serial.println("mBadeand   \t" + String(mBadeand) + " [g]");                  // Pinter tekst, TAB, verdien omgjort til en String, tekst + linjeskift
  Serial.println("lengdeKArm \t" + String(lengdeKatapultarm) + " [m]");         // Pinter tekst, TAB, verdien omgjort til en String, tekst + linjeskift
  Serial.println("pi         \t" + String(pi) + "\n");                          // Pinter tekst, TAB, verdien omgjort til en String, tekst + linjeskift

  float aksTid = (t1 - t0) / 1000.0;                                                                             // [s]       Utfører matte med variablene, lagrer i float
  float totTid = (t2 - t0) / 1000.0;                                                                             // [s]       Utfører matte med variablene, lagrer i float
  float flytid = (t2 - t1) / 1000.0;                                                                             // [s]       Utfører matte med variablene, lagrer i float
  float vinkelRadiusRad = utgangsvinkel * (pi/180);                                                              // [rad]     Utfører matte med variablene, lagrer i float
  float vinkelhastighet = vinkelRadiusRad / aksTid;                                                              // [rad/s]   Utfører matte med variablene, lagrer i float
  float vinkelakselerasjon = vinkelhastighet / aksTid;                                                           // [rad/s^2] Utfører matte med variablene, lagrer i float
  float akselerasjon = lengdeKatapultarm * vinkelakselerasjon;                                                   // [m/s^2]   Vinkelakselerasjon.  Utfører matte med variablene, lagrer i float
  float Fkast = (mBadeand / 1000) * akselerasjon;                                                                // [N]       Utfører matte med variablene, lagrer i float
  float utgangshastighet = akselerasjon * (aksTid);                                                              // [m/s]     Utfører matte med variablene, lagrer i float
  float horisontalHastighet = distanse / (flytid);                                                               // [m/s]     Utfører matte med variablene, lagrer i float
  float utgangshoyde = lengdeKatapultarm * sin(utgangsvinkel * (pi / 180)) + 0.1;                                // [m]       over bakken (0.1 er høyde til omdreiningspunkt) Utfører matte med variablene, lagrer i float
  float maxBanehoyde = (pow(utgangshastighet, 2) * pow(sin(utgangsvinkel * (pi / 180.0)), 2.0)) / (2.0 * 9.81);  // [m]       Utfører matte med variablene, lagrer i float
            
  tall[0] = aksTid;                                                 // Lagrer variabelen i arrayet på plass 0
  tall[1] = totTid;                                                 // Lagrer variabelen i arrayet på plass 1
  tall[2] = flytid;                                                 // Lagrer variabelen i arrayet på plass 2
  tall[3] = akselerasjon;                                           // Lagrer variabelen i arrayet på plass 3
  tall[4] = Fkast;                                                  // Lagrer variabelen i arrayet på plass 4
  tall[5] = utgangshastighet;                                       // Lagrer variabelen i arrayet på plass 5
  tall[6] = horisontalHastighet;                                    // Lagrer variabelen i arrayet på plass 6
  tall[7] = utgangshoyde;                                           // Lagrer variabelen i arrayet på plass 7
  tall[8] = maxBanehoyde;                                           // Lagrer variabelen i arrayet på plass 8

  for (int n = 0; n <= 8; n++) {                                    // For-loop som printer verdiene i seriell monitor og på LCD-skjermen starter
    Serial.print(dataNavn[n] + "  " + tall[n]);                     // Printer verdien i dataNavn <tellevariabel-n-> + "2 SPACE" + verdien i tall <tellevariabel-m->, i seriell monitor
    lcd.clear();                                                    // Sletter alt som var på LCD-skjermen
    lcd.setCursor(0, 0);                                            // Setter cursoren i LCD-skjermen på plass x, linje y
    lcd.print(dataNavn[n]);                                         // Printer teksten i variabelen dataNavn i <tellevariabel-m->
    lcd.setCursor(0, 1);                                            // Setter cursoren i LCD-skjermen på plass x, linje y
    lcd.print(tall[n]);                                             // Printer verdien i tall <tellevariabel-n->
    switch (n) {                                                    // Switch case med <tellevariabel-m->
      case 0 ... 2:                                                 // Om <tellevariabel-n-> == 0 til 2
        Serial.print(" [s]");                                       // Printer tekst i seriell monitor
        lcd.setCursor(13, 1);                                       // Setter cursoren i LCD-skjermen på plass x, linje y
        lcd.print("[s]");                                           // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor
        break;                                                      // Går ut av switch
      case 3:                                                       // Om <tellevariabel-n-> == 3
        Serial.print("[m/s^2]");                                    // Printer tekst i seriell monitor
        lcd.setCursor(9, 1);                                        // Setter cursoren i LCD-skjermen på plass x, linje y
        lcd.print("[m/s^2]");                                       // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor
        break;                                                      // Går ut av switch
      case 4:                                                       // Om <tellevariabel-n-> == 4
        Serial.print(" [N]");                                       // Printer tekst i seriell monitor
        lcd.setCursor(13, 1);                                       // Setter cursoren i LCD-skjermen på plass x, linje y
        lcd.print("[N]");                                           // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor
        break;                                                      // Går ut av switch
      case 5 ... 6:                                                 // Om <tellevariabel-n-> == 5 til 6
        Serial.print(" [m/s]");                                     // Printer tekst i seriell monitor
        lcd.setCursor(11, 1);                                       // Setter cursoren i LCD-skjermen på plass x, linje y
        lcd.print("[m/s]");                                         // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor
        break;                                                      // Går ut av switch
      case 7 ... 8:                                                 // Om <tellevariabel-n-> == 5 til 6
        Serial.print(" [m]");                                       // Printer tekst i seriell monitor
        lcd.setCursor(13, 1);                                       // Setter cursoren i LCD-skjermen på plass x, linje y
        lcd.print("[m]");                                           // Printer tekst på LCD-skjermen, starter på foregående lcd.setCursor
        break;                                                      // Går ut av switch
    }                                                               // Switch case SLUTT
    Serial.println();                                               // Printer linjeskift i seriell monitor
    delay(2000);                                                    // Venter
  }                                                                 // For-loop SLUTT
  lcd.clear();                                                      // Sletter alt som var på LCD-skjermen

  harKastet = !harKastet;                                           // Inverterer variablen 1->0 eller 0->1
}                                                                   // Void beregninger SLUTT





//
