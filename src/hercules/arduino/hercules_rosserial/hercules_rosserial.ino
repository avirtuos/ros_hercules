#include <QueueList.h>
#include <seeed_pwm.h>
#include <ArduinoHardware.h>
#include <ros.h>

#include <std_msgs/String.h>
#include <ros.h>
#include <NewPing.h>
#include <motordriver_4wd.h>

ros::NodeHandle  nh;

int cStrLen(const char* cstr) {
  int counter = 0;
  while (cstr[counter] != '\0') {
    counter++;
  }
  return counter;
}

bool autoFlag = false;
int absLeftSpeed = 0;
int absRightSpeed = 0;


String lastMotorCtrlMsg = "";
void onMotorCtrlMsg( const std_msgs::String& msg) {
  if (String("auto") == String(msg.data)) {
    autoFlag = true;
    return;
  } else if (cStrLen(msg.data) < 4) {
    autoFlag = false;
    MOTOR.setStop1();
    MOTOR.setStop2();
    lastMotorCtrlMsg = "Too Small";
    lastMotorCtrlMsg.concat(sizeof(msg.data));
    lastMotorCtrlMsg.concat(msg.data);
    return;
  }
  autoFlag = false;
  int i = 0;
  int leftSpeed = msg.data[i++] - 48;
  unsigned char leftDir = ('F' == msg.data[i++]) ? DIRF : DIRR;
  int rightSpeed = msg.data[i++] - 48;
  unsigned char rightDir = 1 - (('F' == msg.data[i++]) ? DIRF : DIRR);

  absLeftSpeed = leftSpeed * (('F' == msg.data[1]) ? 1 : -1);
  absRightSpeed = rightSpeed * (('F' == msg.data[3]) ? 1 : -1);

  if (leftSpeed == 0) {
    MOTOR.setStop1();
  } else {
    MOTOR.setSpeedDir1(leftSpeed, leftDir);
  }

  if (rightSpeed == 0) {
    MOTOR.setStop2();
  } else {
    MOTOR.setSpeedDir2(rightSpeed, rightDir);
  }

  lastMotorCtrlMsg = "";
  lastMotorCtrlMsg.concat("L:"); lastMotorCtrlMsg.concat(leftSpeed);
  lastMotorCtrlMsg.concat(" LD:"); lastMotorCtrlMsg.concat(msg.data[1]);
  lastMotorCtrlMsg.concat(" R:"); lastMotorCtrlMsg.concat(rightSpeed);
  lastMotorCtrlMsg.concat(" RD:"); lastMotorCtrlMsg.concat(msg.data[3]);
  lastMotorCtrlMsg.concat(" RAW:"); lastMotorCtrlMsg.concat(msg.data);
}

ros::Subscriber<std_msgs::String> sub("/hercules/motorCtrl", onMotorCtrlMsg );

std_msgs::String sensorMsg;
ros::Publisher sensorTopic("/hercules/sensors", &sensorMsg);


/**
      Sensors
  Soft Serial

  RX: A2
  TX: A3 (blue)

  Encoders
  D2
  D3

  IR
  A0
  A1

  Echo D11, D12

  RX, TX wifi
*/
//35 encoder ticks = 1 revolution = 267 mm
//Motor 1  - Left - encoder interupt 0
//Motor 2 - right - encoder interupt 1
#define TRIGGER_PIN  11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

int right_encoder_count = 0;
int left_encoder_count = 0;

void right_encoder() {
  right_encoder_count++;
}

void left_encoder() {
  left_encoder_count++;
}

int getRightEncoder() {
  detachInterrupt(1);
  int returnValue = right_encoder_count;
  right_encoder_count = 0;
  attachInterrupt(1, right_encoder, RISING);
  return ceil((returnValue / 35.0) * 26.7);
}

int getLeftEncoder() {
  detachInterrupt(0);
  int returnValue = left_encoder_count;
  left_encoder_count = 0;
  attachInterrupt(0, left_encoder, RISING);
  return ceil((returnValue / 35.0) * 26.7);
}

bool checkRearObstacle() {
  return (digitalRead(A0) == LOW);
}

int getFrontDistance() {
  return sonar.ping() / US_ROUNDTRIP_CM;
}


void setup()
{
  MOTOR.init();
  MOTOR.setStop1();
  MOTOR.setStop2();
  attachInterrupt(0, left_encoder, RISING);
  attachInterrupt(1, right_encoder, RISING);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  nh.initNode();
  nh.advertise(sensorTopic);
  nh.subscribe(sub);
}

long lastPub = 0;
void loop()
{
  int frontDistance = getFrontDistance();
  bool rearObstacle = checkRearObstacle();

  if (autoFlag) {
    if (rearObstacle) {
      lastMotorCtrlMsg = "Auto: Rear Obstacle";
      MOTOR.setSpeedDir1(30, DIRF);
      MOTOR.setSpeedDir2(30, 1 - DIRF);
      absRightSpeed = 30;
      absLeftSpeed = 30;
    } else if (frontDistance != 0 && frontDistance < 30) {
      lastMotorCtrlMsg = "Auto: Front Obstacle";
      MOTOR.setSpeedDir1(20, DIRR);
      MOTOR.setSpeedDir2(20, 1 - DIRR);
      absRightSpeed = -30;
      absLeftSpeed = -30;
    } else if (frontDistance < 40) {
      lastMotorCtrlMsg = "Auto: Avoid Front Obstacle";
      MOTOR.setSpeedDir1(35, DIRF);
      MOTOR.setSpeedDir2(35, 1 - DIRR);
      absRightSpeed = -30;
      absLeftSpeed = 30;
    } else if (frontDistance < 80) {
      lastMotorCtrlMsg = "Auto: Go Slow Obstacle";
      MOTOR.setSpeedDir1(20, DIRF);
      MOTOR.setSpeedDir2(20, 1 - DIRF);
      absRightSpeed = 30;
      absLeftSpeed = 30;
    } else if (frontDistance > 80) {
      lastMotorCtrlMsg = "Auto: All Clear";
      MOTOR.setSpeedDir1(30, DIRF);
      MOTOR.setSpeedDir2(30, 1 - DIRF);
      absRightSpeed = 30;
      absLeftSpeed = 30;
    }
  }

  if ((absRightSpeed > 0 || absLeftSpeed > 0) && frontDistance > 0 && frontDistance < 20) {
    MOTOR.setStop1();
    MOTOR.setStop2();

    lastMotorCtrlMsg = "Stop, front obstacle";
    lastMotorCtrlMsg.concat("R: ");
    lastMotorCtrlMsg.concat(absRightSpeed);
    lastMotorCtrlMsg.concat("L: ");
    lastMotorCtrlMsg.concat(absLeftSpeed);
    lastMotorCtrlMsg.concat("Time: ");
    lastMotorCtrlMsg.concat(millis());
  } else if ((absRightSpeed < 0 || absLeftSpeed < 0) && rearObstacle) {
    MOTOR.setStop1();
    MOTOR.setStop2();
    lastMotorCtrlMsg = "Stop, rear obstacle";
    lastMotorCtrlMsg.concat("R: ");
    lastMotorCtrlMsg.concat(absRightSpeed);
    lastMotorCtrlMsg.concat("L: ");
    lastMotorCtrlMsg.concat(absLeftSpeed);
    lastMotorCtrlMsg.concat("Time: ");
    lastMotorCtrlMsg.concat(millis());
  }

  if (millis() - lastPub > 500) {
    String sensorData = "";
    sensorData.concat("A:");
    sensorData.concat(autoFlag);
    sensorData.concat(";P:");
    sensorData.concat(frontDistance);
    sensorData.concat(";IR:");
    sensorData.concat(rearObstacle);
    sensorData.concat(";RE:");
    sensorData.concat(getRightEncoder());
    sensorData.concat(";LE:");
    sensorData.concat(getLeftEncoder());
    sensorData.concat(";LM:");
    sensorData.concat(lastMotorCtrlMsg);
    sensorMsg.data = sensorData.c_str();
    sensorTopic.publish( &sensorMsg );
    lastPub = millis();
  }
  nh.spinOnce();
}
