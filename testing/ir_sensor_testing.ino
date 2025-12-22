#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define FRONT_IR A3
#define BACK_IR  A2

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(FRONT_IR, INPUT);
  pinMode(BACK_IR, INPUT);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("IR Sensor Test");
  delay(1500);
  lcd.clear();
}

void loop() {
  int frontStatus = digitalRead(FRONT_IR);
  int backStatus  = digitalRead(BACK_IR);

  // Front IR status
  lcd.setCursor(0, 0);
  if (frontStatus == LOW) {
    lcd.print("Front: OBSTACLE ");
  } else {
    lcd.print("Front: CLEAR    ");
  }

  // Back IR status
  lcd.setCursor(0, 1);
  if (backStatus == LOW) {
    lcd.print("Back : OBSTACLE ");
  } else {
    lcd.print("Back : CLEAR    ");
  }

  delay(300);
}
