
#define USE_USBCON
//The above flag is required only for Arduino Micro / Pro Micro based on Leonardo

#include <SoftwareSerial.h>

#include <ros.h>

#include <std_msgs/String.h>
/* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   rightEncoderPinA to pin 2, rightEncoderPinB to pin 4 (or pin 3 see below)
   it doesn't matter which encoder pin you use for A or B

   uses Arduino pullups on A & B channel outputs
   turning on the pullups saves having to hook up resistors
   to the A & B channel outputs

*/

#define rightEncoderPinA  3
#define rightEncoderPinB  4

#define leftEncoderPinA  7
#define leftEncoderPinB  6

#define ONE_ROTATION  6000.0
#define CIRCUMFERENCE 15.4488

#define RIGHT_MOTOR_MIN 1
#define RIGHT_MOTOR_STOP 64
#define RIGHT_MOTOR_MAX 127

#define LEFT_MOTOR_MIN 128
#define LEFT_MOTOR_STOP 192
#define LEFT_MOTOR_MAX 255

volatile long rightEncoderPos = 0;  //MOTOR 1  - ( 1 - reverse - 64 - forward - 127)
volatile long leftEncoderPos = 0;   //MOTOR 2 - ( 128 - reverse - 192 - forward - 255)

// software serial #2: RX = digital pin 8, TX = digital pin 9
// on the Mega, use other pins instead, since 8 and 9 don't work on the Mega
SoftwareSerial saberTooth(9, 8);


ros::NodeHandle  nh;



long lastMotorCtrlTime = 0;
void onMotorCtrlMsg( const std_msgs::String& msg) {
  String statusMsg = "OK";

  if (cStrLen(msg.data) < 2) {
    motorStop();
    statusMsg = "Too Small";

    publishSensorData(statusMsg);
    return;
  }

  byte left = msg.data[0];
  byte right = msg.data[1];

  if ((int)left < LEFT_MOTOR_MIN || left > LEFT_MOTOR_MAX) {
    motorStop();
    statusMsg = "Left Motor Error";
    publishSensorData(statusMsg);
    return;
  } else if ((int)right < RIGHT_MOTOR_MIN || right > RIGHT_MOTOR_MAX) {
    motorStop();
    statusMsg = "Right Motor Error";
    publishSensorData(statusMsg);
    return;
  }

  lastMotorCtrlTime = millis();
  unsigned char leftDir = (left < LEFT_MOTOR_STOP) ? 'R' : 'F';
  unsigned char rightDir = (right < RIGHT_MOTOR_STOP) ? 'R' : 'F';

  publishSensorData(statusMsg);

  motorSpeed(left, right);

}

std_msgs::String sensorMsg;
ros::Publisher sensorTopic("/hercules/sensors", &sensorMsg);

void publishSensorData(String msg) {
  String sensorData = "";
  sensorData.concat("RE:");
  sensorData.concat(getRightEncoder());
  sensorData.concat(";LE:");
  sensorData.concat(getLeftEncoder());
  sensorData.concat(";LM:");
  sensorData.concat(msg);
  sensorMsg.data = sensorData.c_str();
  sensorTopic.publish( &sensorMsg );
}

int getRightEncoder() {
  long localEncoder = rightEncoderPos;
  rightEncoderPos = 0;

  return ceil((localEncoder / ONE_ROTATION) * CIRCUMFERENCE);
}

int getLeftEncoder() {
  long localEncoder = leftEncoderPos;
  leftEncoderPos = 0;

  return ceil((localEncoder / ONE_ROTATION) * CIRCUMFERENCE);
}

ros::Subscriber<std_msgs::String> sub("/hercules/motorCtrl", onMotorCtrlMsg );

void setup() {
  delay(10000);

  pinMode(rightEncoderPinA, INPUT);
  digitalWrite(rightEncoderPinA, HIGH);
  pinMode(rightEncoderPinB, INPUT);
  digitalWrite(rightEncoderPinB, HIGH);

  attachInterrupt(0, dorightEncoder, CHANGE);

  pinMode(leftEncoderPinA, INPUT);
  digitalWrite(leftEncoderPinA, HIGH);
  pinMode(leftEncoderPinB, INPUT);
  digitalWrite(leftEncoderPinB, HIGH);

  attachInterrupt(4, doleftEncoder, CHANGE);

  saberTooth.begin(9600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(sensorTopic);
  nh.subscribe(sub);
}

long lastPub = 0;
void loop() {

  if (millis() - lastMotorCtrlTime > 2000) {
    motorStop();
    lastMotorCtrlTime = millis();
  }

  if (millis() - lastPub > 100) {
    publishSensorData("OK");
    lastPub = millis();
  }
  nh.spinOnce();
}

void dorightEncoder() {
  if (digitalRead(rightEncoderPinA) == digitalRead(rightEncoderPinB)) {
    rightEncoderPos--;  //must be opposite of the left encoder to ensure fwd is positive
  } else {
    rightEncoderPos++;  //must be opposite of the left encoder to ensure fwd is positive
  }
}

void doleftEncoder() {
  if (digitalRead(leftEncoderPinA) == digitalRead(leftEncoderPinB)) {
    leftEncoderPos++;
  } else {
    leftEncoderPos--;
  }
}

void motorStop() {
  saberTooth.write((byte)0);
}

void motorSpeed(byte left, byte right) {
  saberTooth.write(left);
  saberTooth.write(right);
}

int cStrLen(const char* cstr) {
  int counter = 0;
  while (cstr[counter] != '\0') {
    counter++1;
  }
  return counter;
}

