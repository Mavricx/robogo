#include <AFMotor.h>
#include <SoftwareSerial.h>

// Motor channels
AF_DCMotor MotorFR(4);
AF_DCMotor MotorFL(1);
AF_DCMotor MotorBL(2);
AF_DCMotor MotorBR(3);

// Bluetooth pins using A4 / A5
const uint8_t BT_RX_PIN = A0; // HC-05 TX -> Arduino A4 (RX)
const uint8_t BT_TX_PIN = A1; // Arduino A5 (TX) -> HC-05 RX (via voltage divider)

SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); // RX, TX

// Other IO
const int buzPin = 4;
const int ledPin = 8;
int valSpeed = 200;

void setup()
{
  Serial.begin(9600);
  btSerial.begin(9600);
  Serial.println("Bluetooth Ready on A4/A5");

  pinMode(buzPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  SetSpeed(valSpeed);

  MotorFL.run(RELEASE);
  MotorFR.run(RELEASE);
  MotorBL.run(RELEASE);
  MotorBR.run(RELEASE);
}

void loop()
{
  // Read from Bluetooth
  if (btSerial.available())
  {
    char c = btSerial.read();
    Serial.print("BT cmd: ");
    Serial.println(c);
    handleCommand(c);
  }

  // Read from Serial Monitor (optional)
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
      return;
    Serial.print("USB cmd: ");
    Serial.println(c);
    handleCommand(c);
  }
}

void handleCommand(char command)
{
  switch (command)
  {

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
    SetSpeed(valSpeed);
    MotorFL.run(BACKWARD);
    MotorFR.run(FORWARD);
    MotorBL.run(BACKWARD);
    MotorBR.run(FORWARD);
    break;

  case 'R':
    SetSpeed(valSpeed);
    MotorFL.run(FORWARD);
    MotorFR.run(BACKWARD);
    MotorBL.run(FORWARD);
    MotorBR.run(BACKWARD);
    break;

  case 'S':
    MotorFL.run(RELEASE);
    MotorFR.run(RELEASE);
    MotorBL.run(RELEASE);
    MotorBR.run(RELEASE);
    break;

  case 'Y':
    digitalWrite(buzPin, HIGH);
    delay(120);
    digitalWrite(buzPin, LOW);
    break;

  case 'U':
    digitalWrite(ledPin, HIGH);
    break;

  case 'u':
    digitalWrite(ledPin, LOW);
    break;

  case '1':
    SetSpeed(65);
    break;
  case '2':
    SetSpeed(130);
    break;
  case '3':
    SetSpeed(195);
    break;
  case '4':
    SetSpeed(255);
    break;
  }
}

// Speed control helper
void SetSpeed(int val)
{
  valSpeed = val;
  MotorFL.setSpeed(val);
  MotorFR.setSpeed(val);
  MotorBL.setSpeed(val);
  MotorBR.setSpeed(val);
}