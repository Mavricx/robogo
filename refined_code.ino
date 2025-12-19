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

enum BuzzerAction
{
    BUZ_BACKWARD,
    BUZ_RIGHT,
    BUZ_LEFT,
    BUZ_OBJECT
};

#define BUZ_ON true
#define BUZ_OFF false

// NEW: tune turn timing
#define TURN_TIME 600    // ms (how long to pivot left/right)
#define RESCAN_PAUSE 250 // ms (pause after reverse before scanning)

LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);

// Motor channels
AF_DCMotor MotorFR(4);
AF_DCMotor MotorFL(1);
AF_DCMotor MotorBL(2);
AF_DCMotor MotorBR(3);

bool btConnected = false;

// NEW: track current motion so we only auto-avoid when moving forward
enum MotionState
{
    STOPPED,
    FORWARDING,
    BACKWARDING,
    TURNING_LEFT,
    TURNING_RIGHT
};
MotionState motionState = STOPPED;

// Bluetooth pins using A4 / A5
const uint8_t BT_RX_PIN = A0; // HC-05 TX -> Arduino A4 (RX)
const uint8_t BT_TX_PIN = A1; // Arduino A5 (TX) -> HC-05 RX (via voltage divider)

SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); // RX, TX
Servo scanServo;

// Other IO
const int buzPin = 12;
const int ledPin = 13;

const int irFPin = A2;
const int irBPin = A3;

int valSpeed = 50;

void lcdPrintClear(uint8_t col, uint8_t row, const char *text);
void lcdPrintClearNum(uint8_t col, uint8_t row, const char *label, int value);
void SetSpeed(int val);

void moveForward();
void moveBackward();
void moveLeft();
void moveRight();
void stopCar();

long getDistance();
long scanDirection(int angle);

// NEW: auto avoid that returns control to BT after finishing
void autoAvoidObstacle_BT();

// NEW: optional helper to keep turning code consistent
static void pivotLeftMs(unsigned long ms)
{
    moveLeft();
    delay(ms);
    stopCar();
}
static void pivotRightMs(unsigned long ms)
{
    moveRight();
    delay(ms);
    stopCar();
}

void buzzerPlay(BuzzerAction action, bool state)
{

    if (state == BUZ_OFF)
    {
        digitalWrite(buzPin, LOW);
        digitalWrite(ledPin, LOW);
        return;
    }

    switch (action)
    {

    case BUZ_BACKWARD:
        // 🚗 "Beep... Beep..." (reverse truck style)
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(buzPin, HIGH);
            digitalWrite(ledPin, HIGH);
            delay(300);
            digitalWrite(buzPin, LOW);
            digitalWrite(ledPin, LOW);
            delay(200);
        }
        break;

    case BUZ_RIGHT:
        // 👉 "pip–pip" (short + playful)
        digitalWrite(buzPin, HIGH);
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(buzPin, LOW);
        digitalWrite(ledPin, LOW);
        delay(80);
        digitalWrite(buzPin, HIGH);
        digitalWrite(ledPin, HIGH);
        delay(180);
        digitalWrite(buzPin, LOW);
        digitalWrite(ledPin, LOW);
        break;

    case BUZ_LEFT:
        // 👈 "pi–pi–pi" (quick triple)
        for (int i = 0; i < 3; i++)
        {
            digitalWrite(buzPin, HIGH);
            digitalWrite(ledPin, HIGH);
            delay(90);
            digitalWrite(buzPin, LOW);
            digitalWrite(ledPin, LOW);
            delay(60);
        }
        break;

    case BUZ_OBJECT:
        // 🚨 "wee-woo wee-woo" (emergency alert)
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(buzPin, HIGH);
            digitalWrite(ledPin, HIGH);
            delay(150);
            digitalWrite(buzPin, LOW);
            digitalWrite(ledPin, LOW);
            delay(60);
            digitalWrite(buzPin, HIGH);
            digitalWrite(ledPin, HIGH);
            delay(300);
            digitalWrite(buzPin, LOW);
            digitalWrite(ledPin, LOW);
            delay(100);
        }
        break;
    }
}

void setup()
{
    Serial.begin(9600);
    btSerial.begin(9600);

    lcd.init();
    lcd.backlight();

    Serial.println("Bluetooth Ready");

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(buzPin, OUTPUT);
    pinMode(ledPin, OUTPUT);

    pinMode(irFPin, INPUT);
    pinMode(irBPin, INPUT);

    scanServo.attach(SERVO_PIN);

    digitalWrite(ledPin, HIGH);
    buzzerPlay(BUZ_OBJECT, BUZ_ON);
    digitalWrite(ledPin, LOW);

    SetSpeed(valSpeed);
    stopCar();
    motionState = STOPPED;
    showStartupScreens();
}

void showStartupScreens()
{

    lcdPrintClear(0, 0, "   Team ROBOGO ");
    lcdPrintClear(0, 1, "Obstacle Avoider");
    delay(1000);

    lcdPrintClear(0, 0, "  Team ROBOGO:  ");
    lcdPrintClearNum(0, 1, "   Group  ", 2);
    delay(1000);

    lcdPrintClear(0, 0, "Team Members:");
    lcdPrintClear(0, 1, "   Pratik  ");
    delay(700);

    lcdPrintClear(0, 1, "  Priyanshu   ");
    delay(700);

    lcdPrintClear(0, 1, "  Raja Swain  ");
    delay(700);

    lcdPrintClear(0, 1, "   Roshan   ");
    delay(700);

    lcdPrintClear(0, 1, "   Jasmin   ");
    delay(700);

    lcdPrintClear(0, 1, "  Anarjyoti  ");
    delay(700);

    lcdPrintClear(0, 0, "System Ready");
    lcdPrintClear(0, 1, "Starting...");
    delay(800);

    lcdPrintClear(0, 0, "BT Ready to Pair");
    lcdPrintClear(0, 1, "Waiting...");
    delay(1000);
    lcd.clear(); // clean screen before loop starts
}

void handleCommand(char command)
{
    btConnected = true; // Bluetooth is active once we receive anything

    switch (command)
    {
    case 'F':
        lcdPrintClear(0, 0, "BT Forward");
        motionState = FORWARDING;
        moveForward();
        break;

    case 'B':
        lcdPrintClear(0, 0, "BT Backward");
        motionState = BACKWARDING;
        moveBackward();
        break;

    case 'L':
        lcdPrintClear(0, 0, "BT Left");
        motionState = TURNING_LEFT;
        moveLeft();
        break;

    case 'R':
        lcdPrintClear(0, 0, "BT Right");
        motionState = TURNING_RIGHT;
        moveRight();
        break;

    case 'S':
        lcdPrintClear(0, 0, "Stopped");
        motionState = STOPPED;
        stopCar();
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

void lcdPrintClear(uint8_t col, uint8_t row, const char *text)
{
    lcd.setCursor(col, row);
    for (uint8_t i = col; i < LCD_COLS; i++)
        lcd.print(' ');

    lcd.setCursor(col, row);
    lcd.print(text);
}

void lcdPrintClearNum(uint8_t col, uint8_t row, const char *label, int value)
{
    char buffer[17]; // for 16x2 LCD
    snprintf(buffer, sizeof(buffer), "%s%d", label, value);
    lcdPrintClear(col, row, buffer);
}

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
    SetSpeed(valSpeed);
    MotorFL.run(FORWARD);
    MotorFR.run(FORWARD);
    MotorBL.run(FORWARD);
    MotorBR.run(FORWARD);
}
void moveBackward()
{
    SetSpeed(valSpeed);
    MotorFL.run(BACKWARD);
    MotorFR.run(BACKWARD);
    MotorBL.run(BACKWARD);
    MotorBR.run(BACKWARD);
}
void moveLeft()
{
    SetSpeed(valSpeed);
    MotorFL.run(BACKWARD);
    MotorFR.run(FORWARD);
    MotorBL.run(BACKWARD);
    MotorBR.run(FORWARD);
}
void moveRight()
{
    SetSpeed(valSpeed);
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

long scanDirection(int angle)
{
    scanServo.write(angle);
    delay(600); // was 1000; reduce a bit for faster decisions (tune if needed)
    return getDistance();
}

// NEW: Auto avoid routine that *temporarily* takes over, then returns control to BT.
void autoAvoidObstacle_BT()
{
    // Take over
    stopCar();
    motionState = STOPPED;

    // Obstacle detected alert (buzzer + LED handled inside buzzerPlay)
    buzzerPlay(BUZ_OBJECT, BUZ_ON);

    lcdPrintClear(0, 0, "Obstacle!");
    lcdPrintClear(0, 1, "Scanning...");

    long leftDist = scanDirection(150);
    long rightDist = scanDirection(30);
    long centerDist = scanDirection(90);
    (void)centerDist; // centerDist is optional; kept for future logic / debug

    // If we have space on either side -> pivot to the side with more space
    if (leftDist > OBSTACLE_DISTANCE || rightDist > OBSTACLE_DISTANCE)
    {
        if (leftDist >= rightDist)
        {
            lcdPrintClear(0, 0, "Auto Left");
            buzzerPlay(BUZ_LEFT, BUZ_ON);
            pivotLeftMs(TURN_TIME);
        }
        else
        {
            lcdPrintClear(0, 0, "Auto Right");
            buzzerPlay(BUZ_RIGHT, BUZ_ON);
            pivotRightMs(TURN_TIME);
        }
    }
    else
    {
        // All sides blocked -> reverse a bit, then rescan and turn
        lcdPrintClear(0, 0, "Auto Reverse");
        buzzerPlay(BUZ_BACKWARD, BUZ_ON);
        moveBackward();

        unsigned long start = millis();
        while (millis() - start < BACK_TIME)
        {
            // Stop early if rear IR detects obstacle
            if (digitalRead(irBPin) == LOW)
            {
                stopCar();
                lcdPrintClear(0, 1, "Back Blocked!");
                buzzerPlay(BUZ_OBJECT, BUZ_ON);
                break;
            }

            // Still allow BT stop to abort auto (optional safety)
            if (btSerial.available())
            {
                char cmd = btSerial.read();
                if (cmd == 'S')
                {
                    handleCommand(cmd);
                    buzzerPlay(BUZ_OBJECT, BUZ_OFF);
                    return; // give control back immediately
                }
            }
        }

        stopCar();
        delay(RESCAN_PAUSE);

        lcdPrintClear(0, 1, "Rescanning...");
        leftDist = scanDirection(150);
        rightDist = scanDirection(30);

        if (leftDist >= rightDist)
        {
            lcdPrintClear(0, 0, "Auto Left");
            buzzerPlay(BUZ_LEFT, BUZ_ON);
            pivotLeftMs(TURN_TIME);
        }
        else
        {
            lcdPrintClear(0, 0, "Auto Right");
            buzzerPlay(BUZ_RIGHT, BUZ_ON);
            pivotRightMs(TURN_TIME);
        }
    }

    stopCar();
    motionState = STOPPED;

    // Ensure buzzer + LED are off (buzzerPlay handles both)
    buzzerPlay(BUZ_OBJECT, BUZ_OFF);

    // Give control back to BT commands (no need to press F again)
    lcdPrintClear(0, 0, "BT Control Ready");
    lcdPrintClear(0, 1, "Send Command");
}

void loop()
{
    // Always listen for bluetooth commands
    if (btSerial.available())
    {
        char cmd = btSerial.read();
        handleCommand(cmd);
    }

    // If BT is NOT connected: NO detection + NO movement allowed
    if (!btConnected)
    {
        stopCar();
        motionState = STOPPED;
        return;
    }

    // If BT is connected, allow movement.
    // Automatic obstacle avoidance takes over ONLY if moving forward and front IR triggers.
    if (motionState == FORWARDING && digitalRead(irFPin) == LOW)
    {
        autoAvoidObstacle_BT(); // takes over, then returns control to BT
    }
}