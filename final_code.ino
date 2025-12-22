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
const int ledPin = 13;

static void ledBlinkShort(uint16_t onMs = 120)
{
    digitalWrite(ledPin, HIGH);
    delay(onMs);
    digitalWrite(ledPin, LOW);
}

const int irFPin = A3;
const int irBPin = A2;

int valSpeed = 100;

// Soft-start / soft-stop tuning (reduces inrush current -> fewer brownout resets)
// Keep these small/fast so behavior stays the same.
#define SOFT_RAMP_STEP 12    // PWM increment per step (0-255)
#define SOFT_RAMP_DELAY_MS 5 // delay per step

static int appliedSpeed = 0; // last PWM applied to motor shield

static void applyMotorSpeedAll(uint8_t spd)
{
    MotorFL.setSpeed(spd);
    MotorFR.setSpeed(spd);
    MotorBL.setSpeed(spd);
    MotorBR.setSpeed(spd);
}

static void rampMotorSpeedTo(int target)
{
    target = constrain(target, 0, 255);
    int s = appliedSpeed;
    if (s == target)
        return;

    const int dir = (target > s) ? 1 : -1;
    while (s != target)
    {
        s += dir * SOFT_RAMP_STEP;
        if ((dir > 0 && s > target) || (dir < 0 && s < target))
            s = target;
        applyMotorSpeedAll((uint8_t)s);
        delay(SOFT_RAMP_DELAY_MS);
    }
    appliedSpeed = s;
}

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

void setup()
{
    Serial.begin(9600);
    btSerial.begin(9600);

    lcd.init();
    lcd.backlight();

    Serial.println("Bluetooth Ready");

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(ledPin, OUTPUT);

    pinMode(irFPin, INPUT);
    pinMode(irBPin, INPUT);

    scanServo.attach(SERVO_PIN);

    digitalWrite(ledPin, HIGH);
    ledBlinkShort();
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

    lcdPrintClear(0, 1, "  Hridesh ");
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
    delay(3000);
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
        ledBlinkShort();
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

void safeStopAndDelay(uint16_t ms = 40)
{
    stopCar();
    delay(ms); // small neutral gap to kill current spike
}
void SetSpeed(int val)
{
    valSpeed = val;
    // Ramp to new speed to avoid sudden current spikes (esp. when changing from low->high).
    rampMotorSpeedTo(valSpeed);
}

void moveForward()
{
    safeStopAndDelay();
    rampMotorSpeedTo(0);
    MotorFL.run(FORWARD);
    MotorFR.run(FORWARD);
    MotorBL.run(FORWARD);
    MotorBR.run(FORWARD);
    rampMotorSpeedTo(valSpeed);
}

void moveBackward()
{
    safeStopAndDelay();
    rampMotorSpeedTo(0);
    MotorFL.run(BACKWARD);
    MotorFR.run(BACKWARD);
    MotorBL.run(BACKWARD);
    MotorBR.run(BACKWARD);
    rampMotorSpeedTo(valSpeed);
}

void moveRight()
{
    safeStopAndDelay();
    rampMotorSpeedTo(0);
    MotorFL.run(FORWARD);
    MotorFR.run(BACKWARD);
    MotorBL.run(FORWARD);
    MotorBR.run(BACKWARD);
    rampMotorSpeedTo(valSpeed);
}

void moveLeft()
{
    safeStopAndDelay();
    rampMotorSpeedTo(0);
    MotorFL.run(BACKWARD);
    MotorFR.run(FORWARD);
    MotorBL.run(BACKWARD);
    MotorBR.run(FORWARD);
    rampMotorSpeedTo(valSpeed);
}
void stopCar()
{
    // Ensure PWM is at 0 so the next start ramps up cleanly.
    applyMotorSpeedAll(0);
    appliedSpeed = 0;
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
    stopCar();  // ensure motors are OFF
    delay(400); // settle power
    scanServo.write(angle);
    delay(500); // reduced delay
    return getDistance();
}

// NEW: Auto avoid routine that *temporarily* takes over, then returns control to BT.
void autoAvoidObstacle_BT()
{
    // Take over
    stopCar();
    motionState = STOPPED;

    // Obstacle detected alert (LED only)
    ledBlinkShort();

    lcdPrintClear(0, 0, "Obstacle Ahead !");
    lcdPrintClear(0, 1, "Scanning...");

    long leftDist = scanDirection(150);
    delay(500);
    long rightDist = scanDirection(30);
    delay(500);
    long centerDist = scanDirection(90);
    delay(500);
    (void)centerDist; // centerDist is optional; kept for future logic / debug

    // If we have space on either side -> pivot to the side with more space
    if (leftDist > OBSTACLE_DISTANCE || rightDist > OBSTACLE_DISTANCE)
    {
        if (leftDist >= rightDist)
        {
            lcdPrintClear(0, 0, "Auto Left");
            ledBlinkShort();
            pivotLeftMs(TURN_TIME);
        }
        else
        {
            lcdPrintClear(0, 0, "Auto Right");
            ledBlinkShort();
            pivotRightMs(TURN_TIME);
        }
    }
    else
    {
        // All sides blocked -> reverse a bit, then rescan and turn
        lcdPrintClear(0, 0, "Auto Reverse");
        ledBlinkShort();
        moveBackward();

        unsigned long start = millis();
        while (millis() - start < BACK_TIME)
        {
            // Stop early if rear IR detects obstacle
            if (digitalRead(irBPin) == LOW)
            {
                stopCar();
                lcdPrintClear(0, 1, "Back Blocked!");
                ledBlinkShort();
                break;
            }

            // Still allow BT stop to abort auto (optional safety)
            if (btSerial.available())
            {
                char cmd = btSerial.read();
                if (cmd == 'S')
                {
                    handleCommand(cmd);
                    digitalWrite(ledPin, LOW);
                    return; // give control back immediately
                }
            }
        }

        stopCar();
        delay(RESCAN_PAUSE);

        lcdPrintClear(0, 1, "Rescanning...");
        leftDist = scanDirection(160);
        rightDist = scanDirection(20);

        if (leftDist >= rightDist)
        {
            lcdPrintClear(0, 0, "Auto Left");
            ledBlinkShort();
            pivotLeftMs(TURN_TIME);
        }
        else
        {
            lcdPrintClear(0, 0, "Auto Right");
            ledBlinkShort();
            pivotRightMs(TURN_TIME);
        }
    }

    stopCar();
    motionState = STOPPED;

    // Ensure LED is off
    digitalWrite(ledPin, LOW);

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
    // Safety: if moving backward and rear IR triggers, stop immediately
    if (motionState == BACKWARDING && digitalRead(irBPin) == LOW)
    {
        stopCar();
        motionState = STOPPED;
        autoAvoidObstacle_BT(); // take over to avoid obstacle in front
    }
}
