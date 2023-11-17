#include <Servo.h>
#include <AFMotor.h>

// Constants for Ultrasonic Sensor
const int trigPin = 2;
const int echoPin = 13;
const unsigned long timeOut = 60000; // Timeout for pulseIn
const int stopDist = 30; // Stop distance in cm

// Servo
Servo myservo; 
int pos = 0;

// Motors
AF_DCMotor motorFrontLeft(1);
AF_DCMotor motorFrontRight(2);
AF_DCMotor motorBackLeft(3);
AF_DCMotor motorBackRight(4);

void setup() {
   Serial.begin(9600); // Starting Serial Terminal

   myservo.attach(10); // Attaches the servo on pin 10
   myservo.write(pos);

   // Initialize motors
   motorFrontLeft.setSpeed(255);
   motorFrontRight.setSpeed(255);
   motorBackLeft.setSpeed(255);
   motorBackRight.setSpeed(255);

   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) { 
   return microseconds / 29 / 2; 
} 

void loop() {
   // Ultrasonic Sensor
   long duration, inches, cm;
   digitalWrite(trigPin, LOW);
   delayMicroseconds(5);
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPin, LOW);
   duration = pulseIn(echoPin, HIGH, timeOut);
   inches = microsecondsToInches(duration);
   cm = microsecondsToCentimeters(duration);
   Serial.print(inches);
   Serial.print("in, ");
   Serial.print(cm);
   Serial.println("cm");
   delay(100);

   // Servo Movement
   for (pos = 0; pos <= 180; pos += 1) {
      myservo.write(pos);
      delay(15);
   }
   for (pos = 180; pos >= 0; pos -= 1) {
      myservo.write(pos);
      delay(15);
   }

   // Add motor control functions here (goForward, goBackward, etc.)
   // Example: goForward(); // Uncomment to test forward motion
}

// Define your motor control functions (stopCar, goForward, etc.) here
void stopCar() {
  motorFrontLeft.run(RELEASE);
  motorFrontRight.run(RELEASE);
  motorBackLeft.run(RELEASE);
  motorBackRight.run(RELEASE);
}

void goForward() {
  motorFrontLeft.run(FORWARD);
  motorFrontRight.run(FORWARD);
  motorBackLeft.run(FORWARD);
  motorBackRight.run(FORWARD);
}

void goBackward() {
  motorFrontLeft.run(BACKWARD);
  motorFrontRight.run(BACKWARD);
  motorBackLeft.run(BACKWARD);
  motorBackRight.run(BACKWARD);
}

void goLeft() {
  motorFrontLeft.run(BACKWARD);
  motorFrontRight.run(FORWARD);
  motorBackLeft.run(BACKWARD);
  motorBackRight.run(FORWARD);
}

void goRight() {
  motorFrontLeft.run(FORWARD);
  motorFrontRight.run(BACKWARD);
  motorBackLeft.run(FORWARD);
  motorBackRight.run(BACKWARD);
}

int getDistance() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long pulseTime = pulseIn(echoPin, HIGH, timeOut);
  int distance = (float)pulseTime * 340 / 2 / 10000;
  return distance;
}

int checkDirection() {
  int distances[2] = {0, 0};
  int turnDir = 1;
  myservo.write(180);
  delay(500);
  distances[0] = getDistance();
  myservo.write(0);
  delay(1000);
  distances[1] = getDistance();
  // Logic to decide turn direction
  // ...
  return turnDir;
}
