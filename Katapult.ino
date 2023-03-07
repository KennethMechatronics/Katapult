const int dreieDirPin = 12;
const int kasteDirPin = 8;
const int dreieStepPin = 13;
const int kasteStepPin = 9;

const int MS1Dreie = 11;
const int MS2Dreie = 10;
const int MS1Kast = 7;
const int MS2Kast = 6;

int hastghetKast = 3;
int hastighetDreie = 3;
bool dreieRetning = false;
bool kast = false;

const int knappKast = 5;
const int knappOK = 4;
const int knappVerdiOpp = 3;
const int knappVerdiNed = 2;

const int endebryterDrei = A3;
const int endebryterKast = A0;
bool endebryterDreiNaad;
bool endebryterKastNaad;

const int sjokkSensor = A4;
const int laser = A5;

const int pot1 = A1;
const int pot2 = A2;



void setup() {
  for (int i = 6; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }

  for (int j = 2; j <= 5; j++) {
    pinMode(j, INPUT_PULLUP);
  }

  pinMode(laser, OUTPUT);
  digitalWrite(laser, HIGH);

  pinMode(endebryterDrei, INPUT_PULLUP);
  pinMode(endebryterKast, INPUT_PULLUP);

  pinMode(sjokkSensor, INPUT);
  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);

  Serial.begin(9600);

  dreieHastighet();                         // Kjører funksjonen som setter dreiehastigheten, første gang programmet kjører er verdien 3, ut i fra int hastighetDreie
  kasteHastighet();                         // Kjører funksjonen som setter kastehastigheten, første gang programmet kjører er verdien 3, ut i fra int hastighetKast
  digitalWrite(dreieRetning, LOW);
  digitalWrite(kast, false);


}

void loop() {



}

void dreieHastighet() {
/* 
 https://learn.sparkfun.com/tutorials/big-easy-driver-hookup-guide?_ga=2.32813566.1930623739.1678007241-749363297.1654960242
 https://cdn.sparkfun.com/datasheets/Robotics/A4988-Datasheet.pdf
 MS1	MS2	MS3	MS Resolution	Excitation Mode
 L	L	L	Full Step	2 Phase
 H	L	L	Half Step	1-2 Phase
 L	H	L	Quarter Step	W1-2 Phase
 H	H	L	Eigth Step	2W1-2 Phase
 H	H	H	Sixteenth Step	4W1-2 Phase 
 
 *MS3 er koblet til jord og dermed altid LOW.*
*/
  switch (hastighetDreie) {
    case 1:  // FULL STEP
      digitalWrite(MS1Dreie, LOW);
      digitalWrite(MS2Dreie, LOW);
      Serial.println("Dreiehastighet er satt til FULL STEP");
      delay(10);
      break;
    case 2:  // HALF STEP
      digitalWrite(MS1Dreie, HIGH);
      digitalWrite(MS2Dreie, LOW);
      Serial.println("Dreiehastighet er satt til HALF STEP");
      delay(10);
      break;
    case 3:  // QUARTER STEP
      digitalWrite(MS1Dreie, LOW);
      digitalWrite(MS2Dreie, HIGH);
      Serial.println("Dreiehastighet er satt til QUARTER STEP");
      delay(10);
      break;
  }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       

void kasteHastighet() {
/*
 https://learn.sparkfun.com/tutorials/big-easy-driver-hookup-guide?_ga=2.32813566.1930623739.1678007241-749363297.1654960242
 https://cdn.sparkfun.com/datasheets/Robotics/A4988-Datasheet.pdf
 MS1	MS2	MS3	MS Resolution	Excitation Mode
 L	L	L	Full Step	2 Phase
 H	L	L	Half Step	1-2 Phase
 L	H	L	Quarter Step	W1-2 Phase
 H	H	L	Eigth Step	2W1-2 Phase
 H	H	H	Sixteenth Step	4W1-2 Phase   
 
 *MS3 er koblet til jord og dermed altid LOW.*
*/
  switch (hastghetKast) {
    case 1:  // FULL STEP
      digitalWrite(MS1Kast, LOW);
      digitalWrite(MS2Kast, LOW);
      Serial.println("Kastehastighet er satt til FULL STEP");
      delay(10);
      break;
    case 2:  // HALF STEP
      digitalWrite(MS1Kast, HIGH);
      digitalWrite(MS2Kast, LOW);
      Serial.println("Kastehastighet er satt til HALF STEP");
      delay(10);
      break;
    case 3:  // QUARTER STEP
      digitalWrite(MS1Kast, LOW);
      digitalWrite(MS2Kast, HIGH);
      Serial.println("Kastehastighet er satt til QUARTER STEP");
      delay(10);
      break;
  }
}












//
