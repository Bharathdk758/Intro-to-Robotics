/*group 15
Bharath Kumar
Onkar Indurkar
Intro to Robotics
04-27-23
LAB 11
*/

#include <Servo.h>
#include "SimpleRSLK.h"
#include <math.h>
uint16_t cntPerRevolution = 360;
float Pi = 3.14;
float flag =0;
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

float r_wheelSpeed = 28.23;  /*28.23; 42.9; 21.45; 30.03*/
float l_wheelSpeed = 27.167; /* 27.167 */
float arr[171];

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
  myservo.write(85); 
  clearMinMax(sensorMinVal, sensorMaxVal);
}

void loop() {
  waitBtnPressed(LP_RIGHT_BTN);
//  for(int i=0; i<=30; i++) {
//  drivestraight(countForDistance(wheelDiameter, cntPerRevolution,15.25),r_wheelSpeed,l_wheelSpeed);}
//  TurninPlace(90,-1);
  if (isCalibrationComplete == false) {
    simpleCalibrate();
    isCalibrationComplete = true;
  }
  waitBtnPressed(LP_RIGHT_BTN);
  stage1(); 
  }

/********************
Function name:stage1
Description: function to find the location of the robot and move to the center
Input: no input
Return: void
*********************/
void stage1(){
  float posn;
  float mini=400; 
  
  delay(2000);
  servoSweep(0, 170, 1);/*servo sweep and getdistance*/
  for(int i=0;i<=170;i++){
    if(arr[i]<mini){
      mini=arr[i];/*finding the minimum value in the array*/
      posn=i*1.06;/*posn of the min value*/
    }
    }
  if(posn<90) TurninPlace(90-posn,-1);/*turn CW or turn CCW deending on posn*/
  if(posn>90) TurninPlace(posn-90,1);
  delay(300); 
  myservo.write(85);
  delay(30);
  TurninPlace(180,1);/*turn 90 deg four times to get other four sides*/
  float b=getDistance();
  delay(30);
  myservo.write(0);
  delay(300);
  float ass=getDistance();
  delay(300);
  myservo.write(170);
  delay(300);
  float c=getDistance();
  delay(300);
  myservo.write(85);
  float gap2 = (b-mini)/2;
  float gap1;
  drivestraight(countForDistance(wheelDiameter, cntPerRevolution,gap2),r_wheelSpeed,l_wheelSpeed);
  delay(300);
  if(ass>c){/*turn CW or CCW depending on which side is longer*/
    gap1=(ass-c)/2;
    TurninPlace(90,-1);
  }
  if(c>ass){
    gap1=(c-ass)/2;
    TurninPlace(90,1);
  }
  drivestraight(countForDistance(wheelDiameter, cntPerRevolution,gap1),r_wheelSpeed,l_wheelSpeed);
  while(true){
    linefollowing();/*follow the line if line is found*/
    }
  }    

/********************
Function name:stage2
Description: function to do the maze
Input: no input
Return: void

*********************/
void stage2(){
  flag=1;
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);

  int x=searchmax();
  delay(200);
  TurninPlace(90,x);
  delay(200);
  drivestraight(countForDistance(wheelDiameter, cntPerRevolution,15.25),r_wheelSpeed,l_wheelSpeed);
  
  while(true){
    myservo.write(170);
    delay(300);
    float maxi = getDistance();
    delay(500);
    myservo.write(85);
  if (maxi>30.5){
    TurninPlace(90,1);
    digitalWrite(GREEN_LED, HIGH);
    delay(500);
    digitalWrite(GREEN_LED, LOW);
    drivestraight(countForDistance(wheelDiameter, cntPerRevolution,15.25),r_wheelSpeed,l_wheelSpeed);

  }
  delay(500);
  if (getDistance() < 20 && maxi < 30.5)  {
    TurninPlace(90,-1);
    digitalWrite(RED_LED, HIGH);
    delay(500);
    digitalWrite(RED_LED, LOW);
  }
  drivestraight(countForDistance(wheelDiameter, cntPerRevolution,15.25),r_wheelSpeed,l_wheelSpeed);
  }
}
/********************
Function name:searchmax
Description: function to find the maximum distance of the side &amp; store its position
Input: no input
Return: posn
*********************/
int searchmax(){
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
  myservo.write(170);
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
  delayMicroseconds(20); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      /*how long it takes for the pulse to bounce back to the sensor in microseconds*/
  calculatedDistanceCentimeters = echoTime / 58.0;  /*calculate the distance in centimeters*/
  Serial.println(calculatedDistanceCentimeters);
  return calculatedDistanceCentimeters;  
}

/********************************************
* Name: servoSweep
* Description: Moves a servo from a minimum to a maximum degree. Accepts user input for min and
* max degree. User can specifiy degree incrememntation.
* Input: int minDeg (Minimum degree), int maxDeg (Maximum degree), int incDeg (increment degree)
* Return: Void
*********************************************/
void servoSweep(int minDeg, int maxDeg, int incDeg) {
  for(int pos=minDeg; pos<=maxDeg; pos+=incDeg) {
    myservo.write(pos);// tell servo to go to position in variable 'pos'
    arr[pos]=getDistance();
    delay(100);                       
  }                                  
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
Function name:backup
Description:function to drivebackwards when a cliff is detected
Input: no input
Return: void
*********************/
void backup(){
  setMotorDirection (BOTH_MOTORS, MOTOR_DIR_BACKWARD);
  uint16_t leftPulseCount = 0;
  uint16_t rightPulseCount = 0;
  delay(500);
  int x=countForDistance(wheelDiameter, cntPerRevolution,15);
  resetRightEncoderCnt();
  resetLeftEncoderCnt();
  setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed);
  setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed);
  enableMotor(BOTH_MOTORS);

  while (leftPulseCount < x && rightPulseCount < x) {
    leftPulseCount = getEncoderLeftCnt();
    rightPulseCount = getEncoderRightCnt();
    
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
  
  TurninPlace(90,-1);
  if (getDistance() > 20){
    drivestraight(countForDistance(wheelDiameter, cntPerRevolution,30.5),r_wheelSpeed,l_wheelSpeed);
  }
  else{
    TurninPlace(90,-1);
  }
}

/********************
Function name: countForDistance
Description:function to find encoder counts for given distance travelled
Return: temp
*********************/
uint32_t countForDistance(float diam, uint16_t cnt_per_rev, uint32_t distance) {
  float temp = (diam * PI) / cnt_per_rev;
  temp = distance / temp;
  return int(temp);
}

/********************
Function name:linefollowing (also same as stage 2)
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

  float straightSpeed = 40.5;
  float value=0;
  readLineSensor(sensorVal);
  delay(1000);
  value = sensorVal[0]+sensorVal[1]+sensorVal[2]+sensorVal[3]+sensorVal[4]+sensorVal[5]+sensorVal[6]+sensorVal[7]; /*checking if the line is underneath*/
  if (value > 1500){
    enableMotor(BOTH_MOTORS);
  while (true) {/*follow the line if line is found*/
    
    readLineSensor(sensorVal);
    readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);
    uint32_t linePos = getLinePosition(sensorCalVal,lineColor);

    setMotorDirection (BOTH_MOTORS, MOTOR_DIR_FORWARD);
    while(getDistance()<15){/*stop if object is found 15cm ahead AND START PRINTING*/
      disableMotor(BOTH_MOTORS);
      digitalWrite(GREEN_LED, HIGH);
      delay(1000);
      stage2();
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
      setRawMotorSpeed(LEFT_MOTOR, normalSpeed3);
      setRawMotorSpeed(RIGHT_MOTOR, fastSpeed3);
    }
    else if (linePos > 5800 && linePos < 7000) { /* Make an extreme right turn */
       setRawMotorSpeed(LEFT_MOTOR, fastSpeed3);
       setRawMotorSpeed(RIGHT_MOTOR, normalSpeed3);
    }
    else if (linePos > 1200 && linePos < 2200) {  /* Make a left turn */
      setRawMotorSpeed(LEFT_MOTOR, normalSpeed2);
      setRawMotorSpeed(RIGHT_MOTOR, fastSpeed2);
    }
    else if (linePos > 4800 && linePos < 5800) { /* Make a right turn */
      setRawMotorSpeed(LEFT_MOTOR, fastSpeed2);
       setRawMotorSpeed(RIGHT_MOTOR, normalSpeed2);
    }
    else if (linePos > 3800 && linePos < 4800) {  /* Make a slight right turn */
       setRawMotorSpeed(LEFT_MOTOR, fastSpeed);
       setRawMotorSpeed(RIGHT_MOTOR, normalSpeed);
    }
    else if (linePos > 2200 && linePos < 3200) { /* Make a slight left turn */
      setRawMotorSpeed(LEFT_MOTOR, normalSpeed);
      setRawMotorSpeed(RIGHT_MOTOR, fastSpeed);
    }
    else { /* if linepos=3500 robot needs to go straight as line is detected under the middle sensors*/
      setRawMotorSpeed(LEFT_MOTOR, straightSpeed);
      setRawMotorSpeed(RIGHT_MOTOR, straightSpeed);  
    }
    enableMotor(BOTH_MOTORS);
    }
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
Function name: TurninPlace
Description:function to make the robot turn with different angles &amp; directions
Input: degrees &amp; direction
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
    if (rightPulseCount < leftPulseCount) {
    setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed-1.81);/*-1.81, -0.5*/
    setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed+2);/* +1.5 */}
    
    if (leftPulseCount < rightPulseCount){
    setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed+2);/*+1.5*/
    setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed-1.81);/*-1.81, -1*/}
   
//    if (rightPulseCount < leftPulseCount+4) {
//    setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed-2.31);/*-1.81, -0.5*/
//    setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed+3);/* +1.5 */}
//    
//    if (leftPulseCount < rightPulseCount+4){
//    setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed+3);/*+1.5*/
//    setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed-2.31);/*-1.81, -1*/}    
//    
    readLineSensor(sensorVal);
    readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, lineColor);
    float value = 0;
    uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
    for (uint8_t i = 0; i < LS_NUM_SENSORS; i++) {
      value += sensorVal[i]; /*summing the values of all sensors*/
    }
    if (value > 19000 && flag==1) { /* if condition to check the sum of sensor values to disable the
motors*/
    disableMotor(BOTH_MOTORS);/*disable motors*/
    digitalWrite(RED_LED, HIGH);/*turn on RED LED while disabling motor*/
    backup();
    break;
  }
    delay(50);
  }
  stopMotor();
  } 


