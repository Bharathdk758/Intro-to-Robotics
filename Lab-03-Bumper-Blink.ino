/*
Group 15
Bharath Kumar
Onkar Indurkar
Intro to Robotics
02-02-23
LAB 3
*/
#include "SimpleRSLK.h"
#define R RED_LED
#define G GREEN_LED
#define B BLUE_LED
bool hitObstacle;
void setup() {
// put your setup code here, to run once:
setupRSLK();
}
void loop() {
hitObstacle = false;
if((digitalRead(BP_SW_PIN_2) == 0)&&(digitalRead(BP_SW_PIN_1) == 0)&&(digitalRead(BP_SW_PIN_0) == 0)) bump012();
//if bumps 1,2,3 are pressed
if((digitalRead(BP_SW_PIN_2) == 0)&&(digitalRead(BP_SW_PIN_1) == 0)) bump12(); //if bumps 2,3 are pressed
if((digitalRead(BP_SW_PIN_2) == 0)&&(digitalRead(BP_SW_PIN_0) == 0)) bump02(); //if bumps 1,3 are pressed
if((digitalRead(BP_SW_PIN_1) == 0)&&(digitalRead(BP_SW_PIN_0) == 0)) bump01(); //if bumps 1,2 are pressed
if(digitalRead(BP_SW_PIN_0) == 0) bump0(); //if bump 1 is pressed
if(digitalRead(BP_SW_PIN_1) == 0) bump1(); //if bump 2 is pressed
if(digitalRead(BP_SW_PIN_2) == 0) bump2(); //if bump 3 is pressed
if(digitalRead(BP_SW_PIN_3) == 0) bump3(); //if bump 4 is pressed
if(digitalRead(BP_SW_PIN_4) == 0) bump4(); //if bump 5 is pressed
// put your main code here, to run repeatedly:
}
/********************
Function name:bump0
Description: bumper 1 is pressed it blinks the RED on-board LED on and off repeatedly at 1Hz.
Input: no input
Return: void
*********************/
void bump0(){
while (!hitObstacle){
digitalWrite(R, HIGH); // turn the RED LED on (HIGH is the voltage level)
delay(500); // wait for half second
digitalWrite(R, LOW); // turn the RED LED off by making the voltage LOW
delay(500); // wait for half second
for(int y = 0;y<TOTAL_BP_SW;y++){ //running a for loop for all bumpers
/* Check if bump switch was pressed */
if(isBumpSwitchPressed(y) == true){ //if any bumper is pressed
hitObstacle = true;
break;//if bumper is pressed we break the loop
}
}
}
}
/********************
Function name: bump1
Description:n bumper 2 is pressed it blinks the GREEN on-board LED on and off repeatedly at 2Hz.
Input: no input
Return:void
*********************/
void bump1(){
while (!hitObstacle){
digitalWrite(G, HIGH); // turn the GREEN LED on (HIGH is the voltage level)
delay(250); // wait for 250ms
digitalWrite(G, LOW); // turn the GREEN LED off by making the voltage LOW
delay(250); // wait for 250ms
for(int y = 0;y<TOTAL_BP_SW;y++){
/* Check if bump switch was pressed */
if(isBumpSwitchPressed(y) == true){
hitObstacle = true;
break;
}
}
}
}
/********************
Function name:bump2
Description: bumper 3 is pressed it blinks the BLUE on-board LED on and off repeatedly at 4Hz.
Input: no input
Return: void
*********************/
void bump2(){
while (!hitObstacle){
digitalWrite(B, HIGH); //TURN BLUE LED ON BY MAKING VOLTAGE LEVEL HIGH
delay(125); // wait for 125ms
digitalWrite(B, LOW); // turn the LED off by making the voltage LOW
delay(125); // wait for 125ms
for(int y = 0;y<TOTAL_BP_SW;y++){
/* Check if bump switch was pressed */
if(isBumpSwitchPressed(y) == true){
hitObstacle = true;
break;
}
}
}
}
/********************
Function name:bump01
Description: bumpers 1 and 2 are pressed at the same time it blinks the RED and
GREEN onboard LEDs on and off simultaneously and repeatedly at 8Hz.
Input: no input
Return: void
*********************/
void bump01(){
while (!hitObstacle){
digitalWrite(R, HIGH); // turn the RED LED on (HIGH is the voltage level)
digitalWrite(G, HIGH); // turn the Green LED on (HIGH is the voltage level)
delay(62.5);
digitalWrite(R, LOW); // turn the RED LED off by making the voltage LOW
digitalWrite(G, LOW); // turn the Green LED off by making the voltage LOW
delay(62.5);
for(int y = 0;y<TOTAL_BP_SW;y++){
/* Check if bump switch was pressed */
if(isBumpSwitchPressed(y) == true){
hitObstacle = true;
break;
}
}
}
}
/********************
Function name:bump02
Description: bumpers 1 and 3 are pressed at the same time it blinks the RED and BLUE on-board
LEDs on and off simultaneously and repeatedly at 16Hz
Input:no input
Return:void
*********************/
void bump02(){
while (!hitObstacle){
digitalWrite(R, HIGH); // turn the Red LED on (HIGH is the voltage level)
digitalWrite(B, HIGH); // turn the Blue LED on (HIGH is the voltage level)
delay(31.25);
digitalWrite(R, LOW); // turn the Red LED off by making the voltage LOW
digitalWrite(B, LOW); // turn the Blue LED off by making the voltage LOW
delay(31.25);
for(int y = 0;y<TOTAL_BP_SW;y++){
/* Check if bump switch was pressed */
if(isBumpSwitchPressed(y) == true){
hitObstacle = true;
break;
}
}
}
}
/********************
Function name:bump12
Description:n bumpers 2 and 3 are pressed at the same time it blinks
the GREEN and BLUE onboard LEDs on and off simultaneously and repeatedly at 32Hz.
Input:no input
Return: void
*********************/
void bump12(){
while (!hitObstacle){
digitalWrite(B, HIGH); // turn the Blue LED on (HIGH is the voltage level)
digitalWrite(G, HIGH); // turn the Green LED on (HIGH is the voltage level)
delay(15.625);
digitalWrite(B, LOW); // turn the Blue LED off by making the voltage LOW
digitalWrite(G, LOW); // turn the Green LED off by making the voltage LOW
delay(15.625);
for(int y = 0;y<TOTAL_BP_SW;y++){
/* Check if bump switch was pressed */
if(isBumpSwitchPressed(y) == true){
hitObstacle = true;
break;
}
}
}
}
/********************
Function name:bump012
Description: bumpers 1, 2, and 3 are pressed at the same time the RED, GREEN, and BLUE on-board
LEDs should be on and constant
Input:no input
Return:void
*********************/
void bump012(){
while (!hitObstacle){
digitalWrite(B, HIGH); // turn the Blue LED on (HIGH is the voltage level)
digitalWrite(G, HIGH); // turn the Green LED on (HIGH is the voltage level)
digitalWrite(R, HIGH); // turn the Red LED on (HIGH is the voltage level)
delay(250);
for(int y = 0;y<TOTAL_BP_SW;y++){ //running a for loop for all bumpers
/* Check if bump switch was pressed */
if(isBumpSwitchPressed(y) == true){ //if any bumper is pressed
digitalWrite(B, LOW); // turning off Blue LED when the bumper switch is pressed
digitalWrite(G, LOW); // turning off Green LED when the bumper switch is pressed
digitalWrite(R, LOW); // turning off Red LED when the bumper switch is pressed
hitObstacle = true;
break; //if bumper is pressed we break the loop
}
}
}
}
/********************
Function name:bump4
Description:bumper 5 is pressed it should turn off all LEDs
Input:no input
return:void
*********************/
void bump4(){
while (!hitObstacle){
digitalWrite(B, LOW); // turn the Blue LED on (HIGH is the voltage level)
digitalWrite(G, LOW); // turn the Green LED on (HIGH is the voltage level)
digitalWrite(R, LOW); // turn the Red LED on (HIGH is the voltage level)
for(int y = 0;y<TOTAL_BP_SW;y++){
/* Check if bump switch was pressed */
if(isBumpSwitchPressed(y) == true){
digitalWrite(B, LOW); // turning off blue LED when the bumper switch is pressed
digitalWrite(G, LOW); // turning off Green LED when the bumper switch is pressed
digitalWrite(R, LOW); // turning off Red LED when the bumper switch is pressed
hitObstacle = true;
break;
}
}
}
}
/********************
Function name:bump3
Description:bumper 4 is pressed it should show the blink sequence. Each LED (or OFF) pattern
should take 0.63 seconds to complete.
Input:no input
Return:void
*********************/
void bump3()
{
while (!hitObstacle){
digitalWrite(B, LOW);
digitalWrite(G, LOW);
digitalWrite(R, LOW); //off
delay(630);
digitalWrite(B, HIGH);
digitalWrite(G, HIGH);
digitalWrite(R, HIGH); //red blue and green is on
delay(630);
digitalWrite(B, LOW);
digitalWrite(G, LOW);
digitalWrite(R, LOW);
delay(630);
digitalWrite(G, HIGH);
delay(630);
digitalWrite(G, LOW);
digitalWrite(B, HIGH);
delay(630);
digitalWrite(B, LOW);
digitalWrite(R, HIGH);
delay(630);
digitalWrite(R, LOW);
delay(630);
digitalWrite(G, HIGH);
digitalWrite(R, HIGH);
delay(630);
digitalWrite(G, LOW);
digitalWrite(B, HIGH);
delay(630);
digitalWrite(R, LOW);
digitalWrite(G, HIGH);
delay(630);
digitalWrite(B, LOW);
digitalWrite(G, LOW);
delay(630);
for(int y = 0;y<TOTAL_BP_SW;y++){
/* Check if bump switch was pressed */
if(isBumpSwitchPressed(y) == true){
hitObstacle = true;
break;
}
}
}
}
