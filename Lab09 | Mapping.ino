/*
Group 15
Bharath Kumar
Onkar Indurkar
Intro to Robotics
04-06-23
LAB 9
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

float r_wheelSpeed = 28.23;  /*28.23; 42.9; 21.45; 30.03*/
float l_wheelSpeed = 27.167;
float arr[181];

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
  if (isCalibrationComplete == false) {
    simpleCalibrate();
    isCalibrationComplete = true;
  }
  waitBtnPressed(LP_RIGHT_BTN);
  blink(3,1);
  escape();/*escape the arena*/
  }

/********************
Function name:escape
Description: function to find the location of the robot and move to the center  
Input: no input
Return: void
*********************/
void escape(){
  int posn;
  float mini=400; 
  
  delay(2000);
  servoSweep(0, 180, 1);/*servo sweep and getdistance*/
  for(int i=0;i<=180;i++){
    if(arr[i]<mini){
      mini=arr[i];/*finding the minimum value in the array*/
      posn=i;/*posn of the min value*/
    }
    }
  if(posn<90) TurninPlace(90-posn,-1);/*turn CW or turn CCW deending on posn*/
  if(posn>90) TurninPlace(posn-90,1);
  if (posn==90);
  delay(500); 
  myservo.write(90);
  delay(500);
  TurninPlace(90,1);/*turn 90 deg four times to get other four sides*/
  float ass=getDistance();
  delay(500);
  TurninPlace(90,1);//2
  float b=getDistance();
  delay(500);
  TurninPlace(90,1);//3
  float c=getDistance();
  delay(500);
  TurninPlace(90,1);//4
  float len=mini+b;/*find length of arena*/
  float wid=ass+c;/*breadth of arena*/
  float gap2 = len/2-mini;
  float gap1;
  float hypotenuse;/*dist from center*/
  float theta;/*angle from center*/
  delay(1000);
  if(ass>c){/*turn CW or CCW depending on which side is longer*/
    gap1=wid/2-c;
    hypotenuse = sqrt(gap1*gap1 + gap2*gap2);
    theta=atan(gap2/gap1);
    theta=theta*57.29;/*radian to degrees*/
    TurninPlace(90+theta,1);
  }
  if(c>ass){
    gap1=wid/2-ass;
    hypotenuse = sqrt(gap1*gap1 + gap2*gap2);
    theta=atan(gap2/gap1);
    theta=theta*57.29;
    TurninPlace(90+theta,-1);
  }
  drivestraight(countForDistance(wheelDiameter, cntPerRevolution,hypotenuse),r_wheelSpeed,l_wheelSpeed);
  while(true){
    TurninPlace(10,1);/*turn until line is found*/
    delay(500);
    linefollowing(mini,ass,b,c,gap1,gap2,hypotenuse,theta);/*follow the line if line is found*/
    }
    }    

/********************
Function name:blink
Description: function to blink Green & blue LED at 1Hz 
Input: counts & led
Return: void
*********************/
void blink(int cnt, int led) {
  for (int i = 0; i < cnt; i++) {
    if (led==1)digitalWrite(GREEN_LED, HIGH);/* turn the LED on (HIGH is the voltage level)*/
    else if(led==2)digitalWrite(BLUE_LED, HIGH);
    delay(500); /* wait for a second*/
    if (led==1)digitalWrite(GREEN_LED, LOW);/* turn the LED off (LOW is the voltage level)*/
    else if(led==2)digitalWrite(BLUE_LED, LOW);
    delay(500);
    }
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

/******************************************** 
 * Name: servoSweep
 * Description: Moves a servo from a minimum to a maximum degree. Accepts user input for min and 
 *              max degree. User can specifiy degree incrememntation.
 * Input: int minDeg (Minimum degree), int maxDeg (Maximum degree), int incDeg (increment degree)
 * Return: Void
 *********************************************/
void servoSweep(int minDeg, int maxDeg, int incDeg) {
  for(int pos=minDeg; pos<=maxDeg; pos+=incDeg) {
    myservo.write(pos);// tell servo to go to position in variable 'pos'
    arr[pos]=getDistance();
    delay(50);                       
  }                                  
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
  resetLeftEncoderCnt();  resetRightEncoderCnt();   
  if (direction==1){/*counterclockwise*/
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD); 
  }
  if (direction==-1){/*clockwise*/
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);  
  }
  enableMotor(BOTH_MOTORS);                         /* "Turn on" the motor*/ 
  setRawMotorSpeed(LEFT_MOTOR, WHEELSPEED_L);         /* Set motor speeds - variable*/ 
  setRawMotorSpeed(RIGHT_MOTOR, WHEELSPEED_R);
  while((l_totalCount<deg*1.6783)||(r_totalCount<deg*2.18)){/*2.13 1.7583*/
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
    if (l_totalCount >= deg*1.6783 && r_totalCount >= deg*2.18) stopMotor();
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

    if (rightPulseCount < leftPulseCount){ 
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed-0.5);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed+1.5);
    }
    if (leftPulseCount < rightPulseCount){
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed+1.5);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed-1);
    }
    if (rightPulseCount < leftPulseCount+2){ 
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed-4);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed+5.5);
    }
    if (leftPulseCount < rightPulseCount+2){
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed+3);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed-1.5);
    }
    if (rightPulseCount < leftPulseCount+4){ 
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed-5.1);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed+6.5);
    }
    if (leftPulseCount < rightPulseCount+4){
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed+6);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed-4.5);
    }
    delay(50);
  }
  stopMotor();
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
  Function name:linefollowing
  Description:function to run the robot on the line until it encounters an object
  Input: no input
  Return: void
*********************/
void linefollowing(float mini,float ass,float b,float c,float gap1,float gap2,float hypotenuse,float theta) {
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
  digitalWrite(BLUE_LED, LOW);
  value = sensorVal[2]+sensorVal[3]+sensorVal[4]+sensorVal[5]; /*checking if the line is underneath*/
  if (value > 4000){
    enableMotor(BOTH_MOTORS);
  while (true) {/*follow the line if line is found*/
    
    readLineSensor(sensorVal);
    readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);
    uint32_t linePos = getLinePosition(sensorCalVal,lineColor);

    setMotorDirection (BOTH_MOTORS, MOTOR_DIR_FORWARD);
    while(getDistance()<19){/*stop if object is found 15cm ahead AND START PRINTING*/
      while(true){
      disableMotor(BOTH_MOTORS);
      Serial.print(" mini: ");
      Serial.print(mini);
      Serial.println(" ");
  
      Serial.print(" a: ");
      Serial.print(ass);
      Serial.println(" ");
  
      Serial.print(" b: ");
      Serial.print(b);
      Serial.println(" ");

      Serial.print(" c: ");
      Serial.print(c);
      Serial.println(" ");
  
      Serial.print(" gap1: ");
      Serial.print(gap1);
      Serial.println(" ");
  
      Serial.print(" gap2: ");
      Serial.print(gap2);
      Serial.println(" ");

      Serial.print(" hypotenuse: ");
      Serial.print(hypotenuse);
      Serial.println(" ");
  
     Serial.print(" theta: ");
     Serial.print(theta);
     Serial.println(" ");
     delay(1000);
      }
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

