#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------------- Ultrasonic ----------------
#define TRIG_PIN 2
#define ECHO_PIN 3
long duration;
float distance;

// ---------------- LCD ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- Motors ----------------
AF_DCMotor MotorFR(4);
AF_DCMotor MotorFL(1);
AF_DCMotor MotorBL(2);
AF_DCMotor MotorBR(3);

// ---------------- Bluetooth ----------------
const uint8_t BT_RX_PIN = A0;
const uint8_t BT_TX_PIN = A1;
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// ---------------- Other IO ----------------
const int buzPin = 4;
const int ledPin = 8;
int valSpeed = 200;

// Obstacle limit
const int STOP_DISTANCE = 20; // cm

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(buzPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Distance:");

  SetSpeed(valSpeed);

  stopMotors();
}

// ---------------- Main Loop ----------------
void loop() {
  readDistance();
  displayDistance();

  // Auto stop if obstacle is close
  if (distance <= STOP_DISTANCE && distance > 0) {
    stopMotors();
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  // Bluetooth commands
  if (btSerial.available()) {
    char c = btSerial.read();
    handleCommand(c);
  }
}

// ---------------- Ultrasonic Functions ----------------
void readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 25000); // timeout added
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void displayDistance() {
  lcd.setCursor(0, 1);
  lcd.print(distance);
  lcd.print(" cm     ");
}

// ---------------- Motor Control ----------------
void stopMotors() {
  MotorFL.run(RELEASE);
  MotorFR.run(RELEASE);
  MotorBL.run(RELEASE);
  MotorBR.run(RELEASE);
}

// ---------------- Command Handler ----------------
void handleCommand(char command) {

  // Block forward motion if obstacle detected
  if (command == 'F' && distance <= STOP_DISTANCE && distance > 0) {
    stopMotors();
    return;
  }

  switch (command) {
    case 'F':
      SetSpeed(valSpeed);
      MotorFL.run(FORWARD);
      MotorFR.run(FORWARD);
      MotorBL.run(FORWARD);
      MotorBR.run(FORWARD);
      break;

    case 'B':
      SetSpeed(valSpeed);
      MotorFL.run(BACKWARD);
      MotorFR.run(BACKWARD);
      MotorBL.run(BACKWARD);
      MotorBR.run(BACKWARD);
      break;

    case 'L':
      MotorFL.run(BACKWARD);
      MotorFR.run(FORWARD);
      MotorBL.run(BACKWARD);
      MotorBR.run(FORWARD);
      break;

    case 'R':
      MotorFL.run(FORWARD);
      MotorFR.run(BACKWARD);
      MotorBL.run(FORWARD);
      MotorBR.run(BACKWARD);
      break;

    case 'S':
      stopMotors();
      break;

    case 'Y':
      digitalWrite(buzPin, HIGH);
      delay(200);
      digitalWrite(buzPin, LOW);
      break;

    case 'U': digitalWrite(ledPin, HIGH); break;
    case 'u': digitalWrite(ledPin, LOW); break;

    case '1': SetSpeed(65); break;
    case '2': SetSpeed(130); break;
    case '3': SetSpeed(195); break;
    case '4': SetSpeed(255); break;
  }
}

// ---------------- Speed Helper ----------------
void SetSpeed(int val) {
  valSpeed = val;
  MotorFL.setSpeed(val);
  MotorFR.setSpeed(val);
  MotorBL.setSpeed(val);
  MotorBR.setSpeed(val);
}