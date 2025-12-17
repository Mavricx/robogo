#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define LCD_COLS 16 // change to 20 if using 20x4
#define LCD_ROWS 2
#define SERVO_PIN 9
#define TRIG_PIN 2
#define ECHO_PIN 10
#define OBSTACLE_DISTANCE 20 // cm
#define BACK_TIME 800        // ms

LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);

// Motor channels
AF_DCMotor MotorFR(4);
AF_DCMotor MotorFL(1);
AF_DCMotor MotorBL(2);
AF_DCMotor MotorBR(3);

bool btConnected = false;
bool forwardCommandActive = false;



// Bluetooth pins using A4 / A5
const uint8_t BT_RX_PIN = A0; // HC-05 TX -> Arduino A4 (RX)
const uint8_t BT_TX_PIN = A1; // Arduino A5 (TX) -> HC-05 RX (via voltage divider)

SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); // RX, TX
Servo scanServo;



// Other IO
const int buzPin = 4;
const int ledPin = 8;

const int irFPin = A2;
const int irBPin = A3;

int valSpeed = 50;

void setup()
{
    Serial.begin(9600);
    btSerial.begin(9600);

    // setting up lcd
    lcd.init();
    lcd.backlight();

    Serial.println("Bluetooth Ready");
    lcdPrintClear(0, 0, "BT Ready to Pair");

    // setting up ultrasonic sensor
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // setting up buzzer and led
    pinMode(buzPin, OUTPUT);
    pinMode(ledPin, OUTPUT);

    // setting up ir sensors
    pinMode(irFPin, INPUT);
    pinMode(irBPin, INPUT);

    // initial message.
    lcdPrintClear(0, 0, "Hi Priyanshu ^_^");
    scanServo.attach(SERVO_PIN);

    SetSpeed(valSpeed);
    stopCar();
}
void handleCommand(char command)
{
    btConnected = true;   // Bluetooth is active

    switch (command)
    {
    case 'F':
        forwardCommandActive = true;
        lcdPrintClear(0, 0, "BT Forward");
        moveForward();
        break;

    case 'S':
        forwardCommandActive = false;
        lcdPrintClear(0, 0, "Stopped");
        stopCar();
        break;

    // ❌ Ignore manual L, R, B to avoid conflict
    case 'L':
    case 'R':
    case 'B':
        // ignored intentionally
        break;

    case 'Y':
        digitalWrite(buzPin, HIGH);
        delay(150);
        digitalWrite(buzPin, LOW);
        break;

    case 'U':
        digitalWrite(ledPin, HIGH);
        break;

    case 'u':
        digitalWrite(ledPin, LOW);
        break;

    case '1': SetSpeed(65); break;
    case '2': SetSpeed(130); break;
    case '3': SetSpeed(195); break;
    case '4': SetSpeed(255); break;
    }
}



// function to print in lcd
void lcdPrintClear(uint8_t col, uint8_t row, const char *text)
{
    // Clear the line from col to end
    lcd.setCursor(col, row);
    for (uint8_t i = col; i < LCD_COLS; i++)
    {
        lcd.print(' ');
    }

    // Print new text
    lcd.setCursor(col, row);
    lcd.print(text);
}

// function to print in lcd with value.
void lcdPrintClearNum(uint8_t col, uint8_t row, const char *label, int value)
{
    char buffer[17]; // for 16x2 LCD
    snprintf(buffer, sizeof(buffer), "%s%d", label, value);
    lcdPrintClear(col, row, buffer);
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

void moveForward()
{
    lcdPrintClear(0, 0, "Moving Forward");
    MotorFL.run(FORWARD);
    MotorFR.run(FORWARD);
    MotorBL.run(FORWARD);
    MotorBR.run(FORWARD);
}
void moveBackward()
{
    SetSpeed(valSpeed);
    lcdPrintClear(0, 0, " Moving Backward");
    MotorFL.run(BACKWARD);
    MotorFR.run(BACKWARD);
    MotorBL.run(BACKWARD);
    MotorBR.run(BACKWARD);
}
void moveLeft()
{
    SetSpeed(valSpeed);
    lcdPrintClear(0, 0, "Turning Left");
    MotorFL.run(BACKWARD);
    MotorFR.run(FORWARD);
    MotorBL.run(BACKWARD);
    MotorBR.run(FORWARD);
}
void moveRight()
{
    SetSpeed(valSpeed);
    lcdPrintClear(0, 0, "Turning Right");
    MotorFL.run(FORWARD);
    MotorFR.run(BACKWARD);
    MotorBL.run(FORWARD);
    MotorBR.run(BACKWARD);
}
void stopCar()
{
    MotorFL.run(RELEASE);
    MotorFR.run(RELEASE);
    MotorBL.run(RELEASE);
    MotorBR.run(RELEASE);
}

// to get distance from ultrasonic
long getDistance()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0)
        return 300; // no echo
    return duration * 0.034 / 2;
}

// to rotate servomotor
long scanDirection(int angle)
{
    scanServo.write(angle);
    delay(1000);
    return getDistance();
}

//half logic without bluetooth integration--currently redundant
void autoAvoidObstacle(){
    stopCar();
    digitalWrite(ledPin, HIGH);

    // Alert
    digitalWrite(buzPin, HIGH);
    delay(200);
    digitalWrite(buzPin, LOW);

    lcdPrintClear(0,1,"Scanning...");

    long leftDist  = scanDirection(150);
    long rightDist = scanDirection(30);
    long centerDist = scanDirection(90);

    if (leftDist > OBSTACLE_DISTANCE || rightDist > OBSTACLE_DISTANCE){
        if (leftDist >= rightDist)
        {
            lcdPrintClear(0,0,"Auto Left");
            moveLeft();
        }
        else
        {
            lcdPrintClear(0,0,"Auto Right");
            moveRight();
        }
        delay(600);
    }
    else{
        // All sides blocked move back a little and scanning again.
        lcdPrintClear(0,0,"Backing...");
        moveBackward();
        delay(800);

        stopCar();
        delay(300);

        // Re-scan after backing
        leftDist  = scanDirection(150);
        rightDist = scanDirection(30);

        if (leftDist >= rightDist)
            moveLeft();
        else
            moveRight();

        delay(600);
    }

    stopCar();
    digitalWrite(ledPin, LOW);
}


void loop(){
    // Listen for bluetooth commands
    if (btSerial.available()) {
        char cmd = btSerial.read();
        handleCommand(cmd);
    }

    //  If BT not connected or forward not commanded → do nothing
    if (!btConnected || !forwardCommandActive) {
        return;
    }

    //  FRONT OBSTACLE CHECK
    if (digitalRead(irFPin) == LOW) {
        stopCar();
        forwardCommandActive = false;

        lcdPrintClear(0, 0, "Obstacle!");
        digitalWrite(ledPin, HIGH);

        // Alert beep
        digitalWrite(buzPin, HIGH);
        delay(200);
        digitalWrite(buzPin, LOW);

        // Scan
        lcdPrintClear(0, 1, "Scanning...");
        long leftDist   = scanDirection(150);
        long centerDist = scanDirection(90);
        long rightDist  = scanDirection(30);

        // Decision
        if (leftDist > OBSTACLE_DISTANCE || rightDist > OBSTACLE_DISTANCE) {

            if (leftDist >= rightDist) {
                lcdPrintClear(0, 0, "Auto Left");
                moveLeft();
            } else {
                lcdPrintClear(0, 0, "Auto Right");
                moveRight();
            }

            delay(600);
        }
        else {
            // 🔙 Reverse
            lcdPrintClear(0, 0, "Auto Reverse");
            moveBackward();

            unsigned long start = millis();
            while (millis() - start < BACK_TIME) {
                if (digitalRead(irBPin) == LOW) {
                    stopCar();
                    lcdPrintClear(0, 1, "Back Blocked!");
                    break;
                }
            }
        }

        stopCar();
        digitalWrite(ledPin, LOW);
        lcdPrintClear(0, 1, "Press F Again");
    }
}

