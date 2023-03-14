#include <Wire.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);
//               rs en  d4 d5 d6 d7

int data;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  Wire.begin(2);

  lcd.begin(16, 2);
  lcd.clear();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Katapult");
  delay(1000);
  lcd.clear();



  //Wire.onReceive(receiveEvent);
}



void loop() {
  Wire.onReceive(receiveEvent);
  delay(500);

}


void receiveEvent() {
    lcd.clear();
  // read one character from the I2C
  data = Wire.read();
  // Print value of incoming data
  lcd.setCursor(0, 0);
  lcd.print("Utg.kast ");
  lcd.print(data);

  lcd.setCursor(0, 1);
  lcd.print("Retning ");
  lcd.print(data);
 
}


//