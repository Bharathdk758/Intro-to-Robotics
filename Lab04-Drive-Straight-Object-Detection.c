/*
* Group 15
* Onkar Indurkar
* Bharat Kumar
* 02 / 09 / 2023
* Lab 4
*/
#include "SimpleRSLK.h"
uint16_t WHEELSPEED_L = 11; // setting initial wheel speed of left motor
speed
uint16_t WHEELSPEED_R = 11.965; // setting initial wheel speed of
right motor speed
uint16_t Lstr_totalCount; // global variaable for storing encoder
counts from Left motor encoder
uint16_t Rstr_totalCount; // global variaable for storing encoder
counts from right motor encoder
void setup() {
// put your setup code here, to run once:
//Serial.begin(9600);
setupWaitBtn(LP_RIGHT_BTN);
setupRSLK();
}
void loop() {
// put your main code here, to run repeatedly:
waitBtnPressed(LP_RIGHT_BTN);
delay(2000); // initial delay after pressing the button
Straight(); // calling function Straight to drive straight till an
object is detected in path
delay(3000); // delay after detecting an object in path
rotate(); // calling function rotate turn 180° CCW
delay(1000); // delay 1sec after rotation
driveback(); // calling func driveback to drive straight to original
position
delay(1000); // wait 1sec
rotate(); // calling function rotate turn 180° CCW
}
/********************
Function name:Straight
Description: Function to drive straight untill it detects an object in its
path
Input: no input
Return: void
*********************/
void Straight() {
resetLeftEncoderCnt(); resetRightEncoderCnt();
while(true){
setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to
drive forward
enableMotor(BOTH_MOTORS); // "Turn on" the motor
setMotorSpeed(RIGHT_MOTOR,WHEELSPEED_R);
setMotorSpeed(LEFT_MOTOR,WHEELSPEED_L); // Set motor speed
if((digitalRead(BP_SW_PIN_2) == 0)||(digitalRead(BP_SW_PIN_3) == 0)){
Lstr_totalCount = getEncoderLeftCnt();
Rstr_totalCount = getEncoderRightCnt();
// Serial.println(Lstr_totalCount);
// Serial.println(Rstr_totalCount);
disableMotor(BOTH_MOTORS);
break;
}
}
}
/********************
Function name:rotate
Description: Function to rotate 180° CCW in its place
Input: no input
Return: void
*********************/
void rotate(){
uint16_t l_totalCount = 0;
uint16_t r_totalCount = 0;
resetLeftEncoderCnt(); resetRightEncoderCnt(); // Set encoder pulse
count back to 0
setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD); // Cause the robot to
drive forward
enableMotor(BOTH_MOTORS); // "Turn on" the
motor
setMotorSpeed(LEFT_MOTOR, WHEELSPEED_L); // Set motor speeds -
variable,
setMotorSpeed(RIGHT_MOTOR, WHEELSPEED_R);
while((l_totalCount<345)||(r_totalCount<360)){
l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
if (l_totalCount >= 345) disableMotor(LEFT_MOTOR); // as left motor is
faster in our case it needs to turn less than 360°
if (r_totalCount >= 360) disableMotor(RIGHT_MOTOR);
}
}
/********************
Function name:rotate
Description: Function to drive straight to its original position using ecoder
counts counted & stored from previous "Straight" function
Input: no input
Return: void
*********************/
void driveback(){
uint16_t l_totalCount = 0;
uint16_t r_totalCount = 0;
resetLeftEncoderCnt(); resetRightEncoderCnt(); // Set encoder pulse count
back to 0
setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
enableMotor(BOTH_MOTORS);// "Turn on" the motor
setMotorSpeed(LEFT_MOTOR, WHEELSPEED_L); // Set motor speeds -
variable,
setMotorSpeed(RIGHT_MOTOR, WHEELSPEED_R);
while((l_totalCount<Lstr_totalCount)||(r_totalCount<Rstr_totalCount)){
l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
if (l_totalCount >= Lstr_totalCount) disableMotor(LEFT_MOTOR);
if (r_totalCount >= Rstr_totalCount) disableMotor(RIGHT_MOTOR);
}
}
