/*group 15
Bharath Kumar
Onkar Indurkar
Intro to Robotics
05-10-23
GRADLAB
*/
#include <Servo.h>
#include "SimpleRSLK.h"
uint16_t cntPerRevolution = 360;
uint16_t WHEELSPEED_L = 18.62;
uint16_t WHEELSPEED_R = 19.8;
uint8_t lineColor = DARK_LINE;
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
float Pi = 3.14;
float wheelDiameter = 7; /*in centimeters*/
bool isCalibrationComplete = false;
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
  myservo.write(85); 
  clearMinMax(sensorMinVal, sensorMaxVal);
}

void loop() {
  waitBtnPressed(LP_RIGHT_BTN);
  if (isCalibrationComplete == false) {
    simpleCalibrate();
    isCalibrationComplete = true;
    }
  waitBtnPressed(LP_RIGHT_BTN);
  blink(3,2);/*blue LED blink*/
  wayout();
}

/********************
Function name: wayout
Description: function to get out of the corridor
Input: none
Return: none
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
    if(arr[i][0]>50)blink(1,1);/*blinking the green light if opening found*/
    
    delay(300);
    myservo.write(170); /*turn left*/
    delay(300); 
    arr[i][1]=getDistance();/*in the 1st index left is stored, when coming back it becomes right*/
    if(arr[i][1]>50) blink(1,1);/*blinking the green light if opening found*/
    
    delay(300);
    myservo.write(90); 
    delay(300);
    drivestraight(countForDistance(wheelDiameter, cntPerRevolution,30.5),r_wheelSpeed,l_wheelSpeed);/*drives straight to next location*/
    delay(300);
  }
  TurninPlace(180,1);
  delay(300);
  int i_prev=4;/*this represents the current position of the robot*/
  for(int i=3;i>=0;i--){/*searching the 2D array in reverse*/
    for(int j=0;j<=1;j++){
      if(arr[i][j]>50){/*if opening exists*/
        delay(300);
        drivestraight(countForDistance(wheelDiameter, cntPerRevolution, (i_prev-i)*30.5),r_wheelSpeed,l_wheelSpeed);/*drive straight to distance of opening*/  
        delay(300);
        if(j==0)TurninPlace(90,1);/*turn to direction of opening*/
        if(j==1) TurninPlace(90,-1);
        delay(300);
        linedetection(countForDistance(wheelDiameter, cntPerRevolution, 100),r_wheelSpeed,l_wheelSpeed);/*escape the corridor BUT if cliff detected comes back to center*/
        delay(300);
        if(j==0)TurninPlace(90,-1);/*turn back to face the direction of corridor*/
        if(j==1) TurninPlace(90,1);
        i_prev=i;/*updating the position with our current position*/
        }
      }
    }
  }

/********************
Function name: getDistance
Description: function to get the distance of an object in front of the ultrasonic sensor
Input: none
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
Description:function to make the robot turn with different angles &amp; directions
Input: degrees and direction
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
Description:function to make the robot go straight for given distance
input: distance to travel, left and right wheel speed
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

/********************
Function name: stopMotor
Description: function to disable motors
Input: none
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
Function name:backup
Description:function to drivebackwards when a cliff is detected
Input: no input
Return: void
*********************/
void backup(){
  setMotorDirection (BOTH_MOTORS, MOTOR_DIR_BACKWARD);/*set motor direction to reverse*/
  uint16_t leftPulseCount = 0;
  uint16_t rightPulseCount = 0;
  delay(500);
  int x=countForDistance(wheelDiameter, cntPerRevolution,13);/*find number of pulses for 13 cm*/
  resetRightEncoderCnt();
  resetLeftEncoderCnt();
  setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed);
  setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed);
  enableMotor(BOTH_MOTORS);

  while (leftPulseCount < x && rightPulseCount < x) {/*travel a distance of 13cm(returning to center of the block)*/
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
}

/********************
Function name:linedtection
Description:function to make the robot go straight  while looking out for cliffs and backing up when a cliff is found 
input: distance to travel, left and right wheel speed
Return: void
*********************/
void linedetection(int x, float r_wheelSpeed, float l_wheelSpeed){
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
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed-2);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed+2);
    }
    if (leftPulseCount < rightPulseCount){
      setRawMotorSpeed (LEFT_MOTOR, l_wheelSpeed+2);
      setRawMotorSpeed (RIGHT_MOTOR, r_wheelSpeed-2);
    }

    readLineSensor(sensorVal);/*reading line sensor value*/
    readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, lineColor);
    float value = 0;
    uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
    for (uint8_t i = 0; i < LS_NUM_SENSORS; i++) {
      value += sensorVal[i]; /*summing the values of all sensors*/
    }
    if (value > 19000) { /* checking if the cliff is detected*/
      disableMotor(BOTH_MOTORS);/*disable motors*/
      backup();
      break;
    }
    
    delay(40);
  }
  stopMotor();
  } 
