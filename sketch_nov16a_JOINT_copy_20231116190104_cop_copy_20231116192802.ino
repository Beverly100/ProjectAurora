// CODE FOR ULTRASONIC SENSOR

const int trigPin = 2; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 13; // Echo Pin of Ultrasonic Sensor
//SENSOR
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
//SERVO

#include <AFMotor.h>

// Create motor objects
AF_DCMotor motorFrontLeft(1);    // Motor 1 on M1 and M2
AF_DCMotor motorFrontRight(2);   // Motor 2 on M3 and M4
AF_DCMotor motorBackLeft(3);     // Motor 3 on M5 and M6
AF_DCMotor motorBackRight(4);    // Motor 4 on M7 and M8
//MOTOR


void setup() {
   Serial.begin(9600); // Starting Serial Terminal
// SENSOR

  myservo.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo.write(pos);
//SERVOR

  Serial.begin(9600);           // Initialize serial communication
  Serial.println("Motor test!");

  // Set up the motors
  motorFrontLeft.setSpeed(255);       // Set the speed for the front left motor (adjust as needed)
  motorFrontRight.setSpeed(255);      // Set the speed for the front right motor (adjust as needed)
  motorBackLeft.setSpeed(255);        // Set the speed for the back left motor (adjust as needed)
  motorBackRight.setSpeed(255);       // Set the speed for the back right motor (adjust as needed)
//MOTORS
}
long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) { 

   return microseconds / 29 / 2; 

} 

// FOR SENSOR 
void loop() {
   long duration, inches, cm;
   pinMode(trigPin, OUTPUT);
   digitalWrite(trigPin, LOW);
   delayMicroseconds(5);
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(15);
   digitalWrite(trigPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   inches = microsecondsToInches(duration);
   cm = microsecondsToCentimeters(duration);
   Serial.print(inches);
   Serial.print("in, ");
   Serial.print(cm);
   Serial.print("cm");
   Serial.println();
   delay(100);
//SENSOR

 for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
// SERVO


void stopCar() 
{
  // Stop all motors
  motorFrontLeft.run(RELEASE);
  motorFrontRight.run(RELEASE);
  motorBackLeft.run(RELEASE);
  motorBackRight.run(RELEASE);
}

void goForward() 
{
  // Move all motors forward
  motorFrontLeft.run(FORWARD);
  motorFrontRight.run(FORWARD);
  motorBackLeft.run(FORWARD);
  motorBackRight.run(FORWARD);
}

void goBackward() 
{
  // Move all motors backward
  motorFrontLeft.run(BACKWARD);
  motorFrontRight.run(BACKWARD);
  motorBackLeft.run(BACKWARD);
  motorBackRight.run(BACKWARD);
}

void goLeft() 
{
  // Turn left by running left motors backward and right motors forward
  motorFrontLeft.run(BACKWARD);
  motorFrontRight.run(FORWARD);
  motorBackLeft.run(BACKWARD);
  motorBackRight.run(FORWARD);
}

void goRight() 
{
  // Turn right by running left motors forward and right motors backward
  motorFrontLeft.run(FORWARD);
  motorFrontRight.run(BACKWARD);
  motorBackLeft.run(FORWARD);
  motorBackRight.run(BACKWARD);
}
// FOR MOTOR



int getDistance() {                                   //Measure the distance to an object{
  unsigned long pulseTime;                          //Create a variable to store the pulse travel time
  int distance;                                     //Create a variable to store the calculated distance
  digitalWrite(trig, HIGH);                         //Generate a 10 microsecond pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  pulseTime = pulseIn(echo, HIGH, timeOut);         //Measure the time for the pulse to return
  distance = (float)pulseTime * 340 / 2 / 10000;    //Calculate the object distance based on the pulse time
  return distance;
}

int checkDirection(){                                            //Check the left and right directions and decide which way to turn
  int distances [2] = {0,0};                                    //Left and right distances
  int turnDir = 1;                                              //Direction to turn, 0 left, 1 reverse, 2 right
  servoLook.write(180);                                         //Turn servo to look left
  delay(500);
  distances [0] = getDistance();                                //Get the left object distance
  servoLook.write(0);                                           //Turn servo to look right
  delay(1000);
  distances [1] = getDistance();                                //Get the right object distance
  if (distances[0]>=200 && distances[1]>=200)                   //If both directions are clear, turn left
    turnDir = 0;
  else if (distances[0]<=stopDist && distances[1]<=stopDist)    //If both directions are blocked, turn around
    turnDir = 1;
  else if (distances[0]>=distances[1])                          //If left has more space, turn left
    turnDir = 0;
  else if (distances[0]<distances[1])                           //If right has more space, turn right
    turnDir = 2;
  return turnDir;
  }
// SENSOR SERVO