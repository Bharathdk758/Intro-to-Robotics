/*
Group 15
Bharath Kumar
Onkar Indurkar
Intro to Robotics
03-21-23
LAB 7
*/
#include "SimpleRSLK.h"
uint16_t cntPerRevolution = 360;
float Pi = 3.14;
float wheelDiameter = 7; /*in cm*/
uint16_t WHEELSPEED_L = 28.5; //23.75;21.75, 23.75
uint16_t WHEELSPEED_R = 25.74;//21.45, 21.45
const int trigPin = 32;/*trigger pin on the distance sensor*/
const int echoPin = 33;/*echo pin on the distance sensor*/
float MAX_SPEED = 45;
float MAX_SPEED2 = 50;
float MAX_SPEED3 = 55;
float MAX_SPEED4 = 60;
float r_wheelSpeed = 42.9;
float l_wheelSpeed = 38.81;
int first=0;
int last=0;
int arr[36];
int turns=0;

void setup() {
setupRSLK();
Serial.begin (9600); 
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
  delay(1000);
  float angle=escapeArena();/*to find the angle of escape*/
  delay(4000);
  TurninPlace(angle,1);/*turning to the escape angle*/
  digitalWrite(GREEN_LED, HIGH);
  delay(2000);
  digitalWrite(GREEN_LED, LOW);
  drivestraight(1500,r_wheelSpeed,l_wheelSpeed);/*driving out the arena*/
}

/********************
Function name: escapeArena
Description: function to find the angle of escape by scanning the arena
Return: angle of escape
*********************/  
float escapeArena(){
  while (getDistance()<90){/*turn 90 degrees if it faces an object at the start*/
    TurninPlace(90,1);
    turns++;/*note the number of turns*/
  }
  delay(300);
  for(int i=0;i<36;i++){/*turn 10 degrees 36 times*/
    TurninPlace(10,1);
    arr[i]=getDistance();/*check the distance of the object and store in an array*/
    if(arr[i]<=90)digitalWrite(BLUE_LED, HIGH);/*turn LED on if object is found*/
    else if (arr[i]>90) arr[i]=-1;/*objects farther than 90cm are stored as '-1' in the array */

    delay(300); 
    digitalWrite(BLUE_LED, LOW);
  }
  for(int i=0;i<36;i++){
    if(arr[i]!=-1){/*if obect detected*/
      if (first==0)first =(i+1)*10;/*store it in first*/
      else last=(i+1)*10;/*if first is stored, update last value*/
    }
  }
  float angle= (first+last)/2;/*find avg to find the escape angle*/
  if(turns!=0){
  TurninPlace(90*turns,-1);/*turn back to the original angle*/
  angle=angle+(90*turns);/*add the turns to the angle*/
  }
  return angle;/*return angle*/
  }

/********************
Function name: getDistance
Description: function to get the distance of an object in front of the ultrasonic sensor
Return: distance calculated
*********************/  
float getDistance(){
  float values[9]; /*declaring  the array*/
  int cnt =0;
  while(cnt<9){ /*taking 9 readings*/
    float echoTime;                  
    float calculatedDistanceCentimeters;
  
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(20);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(50); 
    digitalWrite(trigPin, LOW);

    echoTime = pulseIn(echoPin, HIGH);
    delay(10);
    calculatedDistanceCentimeters = echoTime / 58.0;/*calculate distance in centimeters*/
    values[cnt]=calculatedDistanceCentimeters; /*store it in a array*/
    if(values[cnt]>0 )cnt++;  /*removing 0s and outliers*/

    }
  for(int i=0; i<9; i++) { /*bubblesorting the array*/
    for(int j=0; j<8-i; j++) {
      if(values[j] > values[j+1]) {
        float temp = values[j];
        values[j] = values[j+1];
        values[j+1] = temp;
      }
    }
  }
  return values[4];  /*returning the median value*/
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
if (direction==1){/*turn counter clockwise*/
setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
}
if (direction==-1){/*turn clockwise*/
setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);/*wheels turn in the opposite direction*/
setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
}
enableMotor(BOTH_MOTORS); /* "Turn on" the motor*/
setRawMotorSpeed(LEFT_MOTOR, WHEELSPEED_L); /* Set motor speeds - variable*/
setRawMotorSpeed(RIGHT_MOTOR, WHEELSPEED_R);
float dis=(43.98*deg)/360; /*find distance in cm for degrees 44.01 20-30deg error, 43.98*/
float pulses = countForDistance(7, cntPerRevolution, dis );/*find pulses for given distance*/
while((l_totalCount<pulses)||(r_totalCount<pulses)){
l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
if (l_totalCount >= pulses && r_totalCount >= pulses) disableMotor(BOTH_MOTORS);
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
setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed);
setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed);
enableMotor(BOTH_MOTORS);
//enableMotor (RIGHT_MOTOR);
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

setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed);
setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed);
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

