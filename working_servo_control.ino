#include <Servo.h>

Servo myServo;
int servoPin = 9;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);

  Serial.println("Servo Motor Test Started");
}

void loop() {

  // Move LEFT (0 degree)
  myServo.write(0);
  Serial.println("Direction: LEFT");
  Serial.println("Angle: 0 degrees");
  delay(2000);

  // Move to CENTER (90 degree)
  myServo.write(90);
  Serial.println("Direction: CENTER");
  Serial.println("Angle: 90 degrees");
  delay(2000);

  // Move RIGHT (180 degree)
  myServo.write(180);
  Serial.println("Direction: RIGHT");
  Serial.println("Angle: 180 degrees");
  delay(2000);
}


// #include <Servo.h>

// Servo myServo;
// int servoPin = 9;

// void setup() {
//   Serial.begin(9600);
//   myServo.attach(servoPin);
//   Serial.println("Servo Sweep Test Started");
// }

// void loop() {

//   // Sweep LEFT to RIGHT
//   for (int angle = 0; angle <= 180; angle++) {
//     myServo.write(angle);

//     if (angle < 90)
//       Serial.println("Moving RIGHT");
//     else if (angle > 90)
//       Serial.println("Moving LEFT");
//     else
//       Serial.println("CENTER");

//     Serial.print("Angle: ");
//     Serial.println(angle);

//     delay(20);
//   }

//   delay(1000);

//   // Sweep RIGHT to LEFT
//   for (int angle = 180; angle >= 0; angle--) {
//     myServo.write(angle);

//     Serial.print("Angle: ");
//     Serial.println(angle);

//     delay(20);
//   }

//   delay(1000);
// }
