/*
Group 15
Bharath Kumar
Onkar Indurkar
Intro to Robotics
03-14-23
LAB 5
*/
#include "SimpleRSLK.h"
uint16_t cntPerRevolution = 360;
float Pi = 3.14;
float wheelDiameter = 7; /*in inches*/
uint16_t WHEELSPEED_L = 38.81;
uint16_t WHEELSPEED_R = 42;
const int trigPin = 32;/*trigger pin on the distance sensor*/
const int echoPin = 33;/*echo pin on the distance sensor*/
float MAX_SPEED = 45;
float MAX_SPEED2 = 50;
float MAX_SPEED3 = 55;
float MAX_SPEED4 = 60;
float r_wheelSpeed = 42.9;
float l_wheelSpeed = 38.81;
int a=0;
int b=0;
void setup() {
setupRSLK();
setupWaitBtn(LP_RIGHT_BTN);/* Right button on Launchpad */
setupWaitBtn(LP_LEFT_BTN);
setupLed(GREEN_LED); /* Green led in rgb led */
setupLed(RED_LED); /* Red led in rgb led */
setupLed(BLUE_LED);
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
}
void loop() {
waitBtnPressed(LP_RIGHT_BTN);
blink(3,2);
shape1();
a=1;/* a becomes one after the first shape is done*/
waitBtnPressed(LP_LEFT_BTN);
blink(3,1);
shape2();
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
Function name: getDistance
Description: function to get the distance of an object in front of the ultrasonic sensor
Return: distance calculated
*********************/
float getDistance(){
float echoTime;
float calculatedDistanceCentimeters; /*variable to store the distance*/
/*send out an ultrasonic pulse that's 10us long */
digitalWrite(trigPin, LOW); /*ensures a clean pulse beforehand*/
delayMicroseconds(2);
digitalWrite(trigPin, HIGH); /*sending a high pulse 10us*/
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
echoTime = pulseIn(echoPin, HIGH); /*use the pulsein command to see how long it
takes for the pulse to bounce back to the sensor in microseconds*/
calculatedDistanceCentimeters = echoTime / 58.0;
return calculatedDistanceCentimeters;
}
/********************
Function name: shape1
Description: function to drive the shape 1 a 45 45 90 triangle starting pos from 90°angle
Return: void
*********************/
void shape1(){/**/
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
91.44),r_wheelSpeed,l_wheelSpeed);
delay(400);
TurninPlace(135,1); /* turning a 45° inside angle so outside angle = 180-45° = 135°*/
delay(400);
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
129.31),r_wheelSpeed,l_wheelSpeed);
delay(400);
TurninPlace(135,1);
delay(400);
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
91.44),r_wheelSpeed,l_wheelSpeed);
delay(400);
TurninPlace(90,1); /* turning a 90° inside angle so outside angle = 180-90° = 90°*/
}
/********************
Function name: shape1
Description: function to drive the shape 2 a parallelogram of sides 26" & 37.36" starting
pos from accute angle towards 26" side
Return: void
*********************/
void shape2(){
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
66.04),r_wheelSpeed,l_wheelSpeed);
a=a+1;
delay(400);
TurninPlace(74.5,1); /* turning a 105.5° inside angle so outside angle = 180-105.5° =
74.5°*/
delay(400);
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
94.89),r_wheelSpeed,l_wheelSpeed);
a=a+1;
delay(400);
TurninPlace(105.5,1); /* turning a 74.5° inside angle so outside angle = 180-74.5° =
105.5°*/
delay(400);
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
66.04),r_wheelSpeed,l_wheelSpeed);
a=a+1;
delay(400);
TurninPlace(74.5,1);
delay(400);
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
94.89),r_wheelSpeed,l_wheelSpeed);
a=a+1;
delay(400);
TurninPlace(105.5,1);
}
/********************
Function name: retrace
input:left and right encoder values
Description: function to retrace the shape travelled if an object detected in front of the
robot
Return: void
*********************/
void retrace(uint16_t left, uint16_t right){
delay(400);
drivestraight((left+right)/2,r_wheelSpeed,l_wheelSpeed);/*the current side with pulses
traversed on this side*/
delay(400);
if(a>3){/*a equal 4 when its on the 4th side*/
TurninPlace(74.5,-1);
delay(400);
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
66.04),r_wheelSpeed,l_wheelSpeed);
delay(400);
}
if(a>2){/*a equal 3 when its on the 3th side*/
TurninPlace(105.5,-1);
delay(400);
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
94.89),r_wheelSpeed,l_wheelSpeed);
}
if(a>1){/*a equal 2 when its on the 2th side*/
delay(400);
TurninPlace(74.5,-1);
delay(400);
drivestraight(countForDistance(wheelDiameter, cntPerRevolution,
66.04),r_wheelSpeed,l_wheelSpeed);
}
delay(400);
TurninPlace(180,-1);
stopMotor();
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
resetLeftEncoderCnt(); resetRightEncoderCnt();
if (direction==1){
setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
}
if (direction==-1){
setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
}
enableMotor(BOTH_MOTORS); /* "Turn on" the motor*/
setRawMotorSpeed(LEFT_MOTOR, WHEELSPEED_L); /* Set motor speeds - variable*/
setRawMotorSpeed(RIGHT_MOTOR, WHEELSPEED_R);
while((l_totalCount<deg*1.5783)||(r_totalCount<deg*2.05)){/*2.13 1.7583*/
l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
if (l_totalCount >= deg*1.5783 && r_totalCount >= deg*2.05) stopMotor();
}
}
/********************
Function name: objectDetected
Description:function to detect an object 10cm from the front of the robot and either stop
the robot or retrace the path travelled
input- distance, left pulsecount, right pulse count
Return: void
********************/
void objectDetected(float dist,float leftPulseCount,float rightPulseCount)
{
while(dist<19 & b==0){/*the object is 10cm away from robot 19 because of the
ultrasonic mount to front of the robot distance*/
setRawMotorSpeed(BOTH_MOTORS, 0);
digitalWrite(RED_LED, HIGH);
if(a>0)
{
b=1;
delay(5000);
TurninPlace(180,1);
retrace(leftPulseCount,rightPulseCount);
}
dist=getDistance();
}
}
/********************
Function name: countForDistance
Description:function to find encoder counts for given distance travelled
Return: temp
*********************/
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, uint32_t distance) {
float temp = (wheel_diam * PI) / cnt_per_rev;
temp = distance / temp;
return int(temp);
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
setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed);
setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed);
enableMotor(LEFT_MOTOR);
enableMotor (RIGHT_MOTOR);
delay(75); /* DO NOT CHANGE THIS DELAY */
while (leftPulseCount < x && rightPulseCount < x) {
leftPulseCount = getEncoderLeftCnt();
rightPulseCount = getEncoderRightCnt();
/*incrementing speeds based on pulsecnts */
if (leftPulseCount < rightPulseCount + 1) l_wheelSpeed++;
if (rightPulseCount < leftPulseCount + 1) r_wheelSpeed++;
if (leftPulseCount < rightPulseCount + 2) l_wheelSpeed+=2;
if (rightPulseCount < leftPulseCount + 2) r_wheelSpeed+=2;
if (leftPulseCount < rightPulseCount + 4) l_wheelSpeed+=3;
if (rightPulseCount < leftPulseCount + 4) r_wheelSpeed+=3;
if (leftPulseCount < rightPulseCount + 8) l_wheelSpeed+=4;
if (rightPulseCount < leftPulseCount + 8) r_wheelSpeed+=4;
/*decrementing speeds based on pulsecnts */
if (l_wheelSpeed >= MAX_SPEED){l_wheelSpeed--; }
if (r_wheelSpeed >= MAX_SPEED){r_wheelSpeed--; }
if (l_wheelSpeed >= MAX_SPEED2){l_wheelSpeed -= 2; }
if (r_wheelSpeed >= MAX_SPEED2){r_wheelSpeed -= 2; }
if (l_wheelSpeed >= MAX_SPEED3){l_wheelSpeed -= 3; }
if (r_wheelSpeed >= MAX_SPEED3){r_wheelSpeed -= 3; }
if (l_wheelSpeed >= MAX_SPEED4){l_wheelSpeed -= 4; }
if (r_wheelSpeed >= MAX_SPEED4){r_wheelSpeed -= 4; }
objectDetected(getDistance(),leftPulseCount,rightPulseCount);
digitalWrite(RED_LED, LOW);
setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed);
setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed);
delay(50);
}
stopMotor();
}
/********************
Function name:blink
Description: function to blink Green & blue LED at 1Hz
Input: counts & led
Return: void
*********************/
void blink(int cnt, int led) {
for (int i = 0; i < cnt; i++) {
if (led==1)digitalWrite(GREEN_LED, HIGH);/* turn the LED on (HIGH is the voltage
level)*/
else if(led==2)digitalWrite(BLUE_LED, HIGH);
delay(500); /* wait for a second*/
if (led==1)digitalWrite(GREEN_LED, LOW);/* turn the LED off (LOW is the voltage
level)*/
else if(led==2)digitalWrite(BLUE_LED, LOW);
delay(500);
}
} 
