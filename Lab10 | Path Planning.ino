/*
Group 15
Bharath Kumar
Onkar Indurkar
Intro to Robotics
04-06-23
LAB 10
*/

#include <Servo.h>
#include "SimpleRSLK.h"
#include <math.h>
uint16_t cntPerRevolution = 360;
float Pi = 3.14;
float wheelDiameter = 7; /*in cm*/
uint16_t WHEELSPEED_L = 18.62;
uint16_t WHEELSPEED_R = 19.8;
const int trigPin = 32;/*trigger pin on the distance sensor*/
const int echoPin = 33;/*echo pin on the distance sensor*/

uint8_t lineColor = DARK_LINE;
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
bool isCalibrationComplete = false;//line following codes

float r_wheelSpeed = 28.23;
float l_wheelSpeed = 27.167;
float arr[21];

Servo myservo; 

void setup() {
  setupRSLK();
  Serial.begin (9600); 
  setupWaitBtn(LP_RIGHT_BTN);/* Right button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  setupLed(GREEN_LED); /* Green led in rgb led */
  setupLed(BLUE_LED);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myservo.attach(38,500,2650);
  myservo.write(90); 
  clearMinMax(sensorMinVal, sensorMaxVal);
}

void loop() {
  waitBtnPressed(LP_RIGHT_BTN);
//  delay(1000);
//  while(true){
//  drivestraight (10000,r_wheelSpeed,l_wheelSpeed);
//  delay(500);
//  }
  if (isCalibrationComplete == false) {
    simpleCalibrate();
    isCalibrationComplete = true;
  }
  waitBtnPressed(LP_RIGHT_BTN);
  linefollowing(); 
  while(true){
  int x=searchmini();
  delay(200);
  TurninPlace(90,x);
  delay(200);
  drivestraight (10000,r_wheelSpeed,l_wheelSpeed);
  }
}
/********************
Function name:searchmini
Description: function to find the largest distance of the wall from 0, 90, 180° scans
Input: void
Return: position
*********************/
int searchmini(){
  myservo.write(0);
  delay(200);
  /*declaring variables*/
  float maxi = getDistance();
  int a;
  int posn =-1;/*saving the position if the max distance is found at 0°*/
  delay(200);
  myservo.write(90);
  delay(200);
  a = getDistance();/*getting the distance of the wall at 90°*/
  if(maxi<a){
    maxi=a;
    posn=0;/*saving the position if the max distance is found at 90°*/
    delay(200);
  }
  myservo.write(180);
  delay(200);
  a = getDistance(); /*getting the distance of the wall at 180°*/
  if(maxi<a){
    maxi=a;
    posn=1; /*saving the position if the max distance is found at 180°*/
    delay(200);
  }
  myservo.write(90);
  return posn;
}

/********************
Function name: getDistance
Description: function to get the distance of an object in front of the ultrasonic sensor
Return: distance calculated
*********************/  
float getDistance(){
  float echoTime; 
  float calculatedDistanceCentimeters; 

  /*send out an ultrasonic pulse that's 10us long */
  digitalWrite(trigPin, LOW); /*ensures a clean pulse beforehand*/
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); /*sending a high pulse 10us*/
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      /*how long it takes for the pulse to bounce back to the sensor in microseconds*/
  calculatedDistanceCentimeters = echoTime / 58.0;  /*calculate the distance in centimeters*/
  Serial.println(calculatedDistanceCentimeters);
  return calculatedDistanceCentimeters;  
}

/********************
Function name: TurninPlace
Description:function to make the robot turn with different angles & directions
Input: degrees & direction
Return: void
*********************/
void TurninPlace(int deg,int direction){
  uint16_t l_totalCount = 0;
  uint16_t r_totalCount = 0;
  float puls=deg*2;
  resetLeftEncoderCnt();  resetRightEncoderCnt();   
  if (direction==1){/*counterclockwise*/
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD); 
  }
  if (direction==-1){/*clockwise*/
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);  
  }
  if (direction==0)return;
  enableMotor(BOTH_MOTORS);                         /* "Turn on" the motor*/ 
  setRawMotorSpeed(LEFT_MOTOR, WHEELSPEED_L);         /* Set motor speeds - variable*/ 
  setRawMotorSpeed(RIGHT_MOTOR, WHEELSPEED_R);
  while((l_totalCount<puls)||(r_totalCount<puls)){
    l_totalCount = getEncoderLeftCnt(); 
    r_totalCount = getEncoderRightCnt();
    if(l_totalCount<r_totalCount){
        setRawMotorSpeed(LEFT_MOTOR, WHEELSPEED_L+2);         /* Set motor speeds - variable*/ 
        setRawMotorSpeed(RIGHT_MOTOR, WHEELSPEED_R-2);
        }
    if(l_totalCount>r_totalCount){
        setRawMotorSpeed(LEFT_MOTOR, WHEELSPEED_L-2);         /* Set motor speeds - variable*/ 
        setRawMotorSpeed(RIGHT_MOTOR, WHEELSPEED_R+2);
    }
  }
  stopMotor();
  }

/********************
  Function name:linefollowing
  Description:function to run the robot on the line until it encounters an object
  Input: no input
  Return: void
*********************/
void linefollowing() {
  float normalSpeed = 11.595; /* ratio = 3.449 */
  uint16_t fastSpeed = 40;

  float normalSpeed2 = 17.5;   /* ratio = 4.5714 */
  uint16_t fastSpeed2 = 80;

  float normalSpeed3 = 5;   /* ratio = 12 */
  uint16_t fastSpeed3 = 60;

  float straightSpeed = 20.5;
  float value=0;
  while (true) {/*follow the line if line is found*/
    readLineSensor(sensorVal);
    readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);
    uint32_t linePos = getLinePosition(sensorCalVal,lineColor);

    setMotorDirection (BOTH_MOTORS, MOTOR_DIR_FORWARD);
    while(getDistance()<15){/*stop if object is found 15cm ahead AND START PRINTING*/
      disableMotor(BOTH_MOTORS);
      return;
    }
    while(linePos==0){/*if line is missed turn until line is found*/
       readLineSensor(sensorVal);
    readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);
      linePos = getLinePosition(sensorCalVal,lineColor);
      TurninPlace(15,-1);
    }

    setMotorDirection (BOTH_MOTORS, MOTOR_DIR_FORWARD);
    enableMotor(BOTH_MOTORS);
    if (linePos > 0 && linePos < 1200) { /* Make an extreme left turn */
      pivotTurn(normalSpeed3, fastSpeed3);
    }
    else if (linePos > 5800 && linePos < 7000) { /* Make an extreme right turn */
      pivotTurn(fastSpeed3,normalSpeed3);
    }
    else if (linePos > 1200 && linePos < 2200) {  /* Make a left turn */
      pivotTurn(normalSpeed2, fastSpeed2);
    }
    else if (linePos > 4800 && linePos < 5800) { /* Make a right turn */
      pivotTurn(fastSpeed2,normalSpeed2);
    }
    else if (linePos > 3800 && linePos < 4800) {  /* Make a slight right turn */
      pivotTurn(fastSpeed,normalSpeed);
    }
    else if (linePos > 2200 && linePos < 3200) { /* Make a slight left turn */
      pivotTurn(normalSpeed, fastSpeed);
    }
    else { /* if linepos=3500 robot needs to go straight as line is detected under the middle sensors*/
      straight(straightSpeed,straightSpeed);
    }
    enableMotor(BOTH_MOTORS);
    digitalWrite(BLUE_LED, HIGH);
    }
 }

/********************
  Function name:simpleCalibrate
  Description:function to calibrate the robot before setting on the track
  Input: no input
  Return: void
*********************/
void simpleCalibrate() {
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD); /* Set both motors direction forward */
  enableMotor(BOTH_MOTORS);/* Enable both motors */
  setMotorSpeed(BOTH_MOTORS, 20); /* Set both motors speed 20 */
  for (int x = 0; x < 100; x++) {
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
  }

  disableMotor(BOTH_MOTORS);/* Disable both motors */
}

/********************
  Function name:pivotTurn
  Description:function to pivot the robot at given speed
  Input: leftwheel speed and right wheel speed
  Return: void
*********************/
void pivotTurn(float leftspd, float rightspd){
  setRawMotorSpeed(LEFT_MOTOR, leftspd);
  setRawMotorSpeed(RIGHT_MOTOR, rightspd);
}

/********************
  Function name:straight
  Description:function to make the robot go straight
  Input: leftwheel speed and right wheel speed
  Return: void
*********************/
void straight(float leftspd, float rightspd){
      setRawMotorSpeed(LEFT_MOTOR, leftspd);
      setRawMotorSpeed(RIGHT_MOTOR, rightspd);  
}

/********************
Function name: stopMotor
Description: function to disable motors
Return: void
*********************/
void stopMotor(){
  disableMotor(BOTH_MOTORS);
}

/********************
Function name:drivestraight
Description:function to make the robot go straight by using wheel encoders and stopping
motors or retracing the shape
Return: void
*********************/
void drivestraight (int x, float r_wheelSpeed, float l_wheelSpeed){
  uint16_t leftPulseCount = 0;
  uint16_t rightPulseCount = 0;
  resetRightEncoderCnt();
  resetLeftEncoderCnt();
  setMotorDirection (BOTH_MOTORS, MOTOR_DIR_FORWARD);

  setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed);
  setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed);
  enableMotor(BOTH_MOTORS);

  while (leftPulseCount < x && rightPulseCount < x) {
    leftPulseCount = getEncoderLeftCnt();
    rightPulseCount = getEncoderRightCnt();
    /*incrementing speeds based on pulsecnts */
    if(getDistance()<15){
      disableMotor(BOTH_MOTORS);
      return;
    }
    if (rightPulseCount < leftPulseCount){ 
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed-2);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed+2);
    }
    if (leftPulseCount < rightPulseCount){
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed+2);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed-2);
    }

    delay(40);
  }
  stopMotor();
  } 

