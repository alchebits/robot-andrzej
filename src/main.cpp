#include <Arduino.h>
#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>


#define HC_SR04_TRIG_PIN A0 // length test
#define HC_SR04_ECHO_PIN A1 // length
#define MAX_DISTANCE 200 // max possible detection distance
#define START_ROTATE_DISTANCE 40 // rotate if distance
#define STOP_ROTATE_DISTANCE 100 // stop rotate if distance == 60
#define LR_SERVO_PIN 9


#define MAX_SPEED 255 // of wheels, 0-255 range
#define MAX_SPEED_OFFSET 40 // diff speed between wheels

AF_DCMotor leftMotor(1);
AF_DCMotor rightMotor(4);
NewPing sonar(HC_SR04_TRIG_PIN, HC_SR04_ECHO_PIN, MAX_DISTANCE); // set up HC-SR04 sensor

Servo servoLR;

#define TIMERS_NUM 5
unsigned long timerStartMillis[TIMERS_NUM];
unsigned int servoUDAngle = 105;
enum TURN_SIDE{
  LEFT = 0,
  RIGHT,
  NONE
};

void resetAllTimers();
void moveForward();
TURN_SIDE lookAround(int dangerousDistanceCM);
void turnOneWheel(TURN_SIDE side);
void stopMotors();
int pingDistanceCM();
void resetTimer(int i);
unsigned long timerMS(int i);
double timerSeconds(int i);


void setup() {
  //Serial.begin(9600);
  servoLR.attach(LR_SERVO_PIN);  // pin 9
  servoLR.write(90);

  resetAllTimers();
  moveForward();
}

TURN_SIDE randTurnSide()
{
  if( (millis()%2) == 0 )
          return TURN_SIDE::LEFT;
        else
          return TURN_SIDE::RIGHT;
}

void loop()
{
    if(timerMS(0) > 4000)
    {
      resetTimer(0);
      stopMotors();
      turnOneWheel(lookAround(START_ROTATE_DISTANCE));
      moveForward();
    }

    if(timerMS(1) > 15000)
    {
      resetTimer(1);
      stopMotors();
      turnOneWheel(randTurnSide());
      moveForward();
    }

    if(pingDistanceCM() <= START_ROTATE_DISTANCE)
    {
      stopMotors();
      TURN_SIDE ts = lookAround(START_ROTATE_DISTANCE);
      if(ts == TURN_SIDE::NONE)
      {
          ts = randTurnSide();
      }
      turnOneWheel(ts);
      moveForward();
    }

    delay(50);
}

TURN_SIDE lookAround(int dangerousDistanceCM)
{
  int ANGLES_NUM = 3;
  int angles[ANGLES_NUM] = {45,135,90};
  int minDistance = 999;
  int tmpDistance;
  int closestDistAngleID = 0;

  for(int i = 0 ; i < ANGLES_NUM ; i++)
  {
    servoLR.write(angles[i]); // look right
    delay(500);
    tmpDistance = pingDistanceCM();
    if(tmpDistance < minDistance)
    {
      minDistance = tmpDistance;
      closestDistAngleID = i;
    }
  }

  if(minDistance <= dangerousDistanceCM )
  {
    if(angles[closestDistAngleID] < 90)
    {
      return TURN_SIDE::LEFT;
    }
    else
    {
      return TURN_SIDE::RIGHT;
    }
  }
  else
  {
    return TURN_SIDE::NONE;
  }
}

void turnOneWheel(TURN_SIDE side)
{
   switch(side)
   {
      case TURN_SIDE::LEFT:
        rightMotor.run(RELEASE);
        leftMotor.run(BACKWARD);
        leftMotor.setSpeed(MAX_SPEED);
      break;
      case TURN_SIDE::RIGHT:
        leftMotor.run(RELEASE);
        rightMotor.run(BACKWARD);
        rightMotor.setSpeed(MAX_SPEED);
      break;
      case TURN_SIDE::NONE:
      default:
        return;
   }

   delay(500);
   resetTimer(2);
   while(pingDistanceCM() < STOP_ROTATE_DISTANCE )
   {
    if(timerMS(2) > 1000)
      break;

    delay(100);
   }
}

void stopMotors()
{
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

void moveForward()
{
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
  leftMotor.setSpeed(MAX_SPEED);
  rightMotor.setSpeed(MAX_SPEED);
}

int pingDistanceCM()
{
  int distance = sonar.ping_cm();
  if(distance <= 0)
    distance = 999;
    return distance;
}

void resetAllTimers()
{
  for(int i = 0 ; i < TIMERS_NUM ; i++)
  {
    resetTimer(i);
  }
}

void resetTimer(int i)
{
  timerStartMillis[i] = millis();
}

unsigned long timerMS(int i)
{
  return millis() - timerStartMillis[i];
}

double timerSeconds(int i)
{
  return (millis() - timerStartMillis[i]) / 1000.0;
}
