/*
Group 15
Bharath Kumar
Onkar Indurkar
Intro to Robotics
03-28-23
LAB 8
*/

#include <Servo.h>
#include "SimpleRSLK.h"
uint16_t cntPerRevolution = 360;
float Pi = 3.14;
float wheelDiameter = 7; /*in cm*/
uint16_t WHEELSPEED_L = 38.81;
uint16_t WHEELSPEED_R = 42;
const int trigPin = 32;/*trigger pin on the distance sensor*/
const int echoPin = 33;/*echo pin on the distance sensor*/

float r_wheelSpeed = 28.53;  
float l_wheelSpeed = 27.167; 
float arr[4][2];

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
}

void loop() {
  waitBtnPressed(LP_RIGHT_BTN);
  blink(3,2);/*blue LED blink*/
  wayout();
}

/********************
Function name: wayout
Description: function to get out of the arena after finding the one way out by saving the position while scanning
input: void
Return: void
*********************/  
void wayout(){
  int maxi=0;
  int posi,posj;
  
  drivestraight(countForDistance(wheelDiameter, cntPerRevolution, 30.5),r_wheelSpeed,l_wheelSpeed);
  delay(300);
  for (int i=0; i<4;i++){
    myservo.write(0); /*turn right*/
    delay(300);
    arr[i][0]=getDistance();/*in the 0th index right is stored, when coming back it becomes left*/
    if(arr[i][0]>50)blink(1,1);/*blink when opening found*/
    
    delay(300);
    myservo.write(170); /*turn left*/
    delay(300); 
    arr[i][1]=getDistance();/*in the 1st index left is stored, when coming back it becomes right*/
    if(arr[i][1]>50)blink(1,1);
    
    delay(300);
    myservo.write(90); 
    delay(300);
    drivestraight(countForDistance(wheelDiameter, cntPerRevolution,30.5),r_wheelSpeed,l_wheelSpeed);/*drives straight to next location*/
    delay(300);
  }
  TurninPlace(180,1);
  delay(300);
  for(int i=0;i<4;i++){/*scanning the 2D array for way out*/
    for(int j=0;j<=1;j++){
      if(arr[i][j]>maxi){
        maxi=arr[i][j];/*finding the escape path*/
        posi=i;/*distance of the escape path*/
        posj=j; /*direction of the escape path*/
        }
      }
    }
  drivestraight(countForDistance(wheelDiameter, cntPerRevolution, (4-posi)*30.5),r_wheelSpeed,l_wheelSpeed);/*driving back along the corridor*/
  delay(300);
  if(posj==0) TurninPlace(90,1);/*1 90 degrees turns left*/
  if(posj==1) TurninPlace(90,-1);/*-1 90 degrees clockwise turns right*/ 
  delay(300);
  drivestraight(countForDistance(wheelDiameter, cntPerRevolution, 100),r_wheelSpeed,l_wheelSpeed);/*escape*/
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
  resetLeftEncoderCnt();  resetRightEncoderCnt();   
  if (direction==1){
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD); 
  }
  if (direction==-1){
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);  
  }
  enableMotor(BOTH_MOTORS);                         /* "Turn on" the motor*/ 
  setRawMotorSpeed(LEFT_MOTOR, WHEELSPEED_L);         /* Set motor speeds - variable*/ 
  setRawMotorSpeed(RIGHT_MOTOR, WHEELSPEED_R);
  while((l_totalCount<deg*1.7583)||(r_totalCount<deg*2.13)){/*2.13 1.7583*/
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
    if (l_totalCount >= deg*1.7583 && r_totalCount >= deg*2.13) stopMotor();
    }
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
Function name: stopMotor
Description: function to disable motors
Return: void
*********************/
void stopMotor(){
  disableMotor(BOTH_MOTORS);
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

/******************************************** 
 * Name: servoSweep
 * Description: Moves a servo from a minimum to a maximum degree. Accepts user input for min and 
 *              max degree. User can specifiy degree incrememntation.
 * Input: int minDeg (Minimum degree), int maxDeg (Maximum degree), int incDeg (increment degree)
 * Return: Void
 *********************************************/
void servoSweep(int minDeg, int maxDeg, int incDeg) {
  for(int pos=minDeg; pos<=maxDeg; pos+=incDeg) {
    myservo.write(pos);    
    delay(50);        
  }

  for(int pos=maxDeg; pos>=minDeg; pos-=incDeg) {
    myservo.write(pos);
    delay(50);       
  }
}  


