#include <Servo.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightDriveMotor;
Servo servo_LeftDriveMotor;
Servo servo_PyramidClaw;
Servo servo_TesseractClaw;
Servo servo_ElevatorMotor;

I2CEncoder encoder_Elevator;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_RightMotor;

//Constant pins
//Digital Pins
const int ci_Ultrasonic_PingB = 2 ; //output plug back US
const int ci_Ultrasonic_DataB = 3 ; //input plug back US
const int ci_Ultrasonic_PingF = 4 ; //output plug front US
const int ci_Ultrasonic_DataF = 5 ; //input plug front US
const int ci_Ultrasonic_Ping_Pyra; // output plug for pyramid US
const int ci_Ultrasonic_Data_Pyra; //input plug for pyramid US
const int ci_Elevator = 6; // output pin for elevator
const int ci_Mode_Switch = 7; // input pin for mode selector
const int ci_Right_Motor = 8; //output pin for right motor
const int ci_Left_Motor = 9 ; // output pin for left motor
const int ci_Pyramid_Claw = 10; // output pin for claw responsible for picking up pyramid
const int ci_Tesseract_Claw = 11; // output pin for claw responsible for picking up power cube

//Analog Pins
const int ci_HallEffect = A0; // input pin for Hall effect sensor
const int ci_I2C_SDA = A4;
const int ci_I2C_SCL = A5;



unsigned long ul_Echo_TimeF; // Echo time of the front ultrasonic
unsigned long ul_Echo_TimeB; // Echo time of the back ultrasoni
unsigned long ul_Echo_TimePyramid; // Echo time of the front ultrasonic for finding the pyramid

int Hall_Reading; // Reading obtained from Hall effect sensor
int servo_Pos_Tess; // Position of the servo responsible for grabbing the tesseract
int servo_Pos_Pyra; // Position of the servo resonsbile for grabbing the pyramid

//Motor Speeds
unsigned long ul_Left_Motor_Speed;
unsigned long ul_Right_Motor_Speed;
unsigned long ul_Elevator_Motor_Speed;

int state_Var = 0; // state variable


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  Serial.begin(2400);

  //Ultrasonics
  pinMode(ci_Ultrasonic_PingF, OUTPUT);
  pinMode(ci_Ultrasonic_DataF, INPUT);
  pinMode(ci_Ultrasonic_PingB, OUTPUT);
  pinMode(ci_Ultrasonic_DataB, INPUT);
  pinMode(ci_Ultrasonic_Ping_Pyramid, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Pyramid, INPUT);

  //Drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightDriveMotor.attach(ci_Right_Motor); //Right drive motor
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftDriveMotor.attach(ci_Left_Motor); // Left Motor

  //Setup claws
  pinMode(ci_Tesseract_Claw, OUTPUT);
  servo_TesseractClaw.attach(ci_Tesseract_Claw);// Tesseract Claw
  pinMode(ci_Pyramid_Claw, OUTPUT);
  servo_PyramidClaw.attach(ci_Pyramid_Claw); // Pyramid Claw

  //Setup Elevator
  pinMode(ci_Elevator, OUTPUT);
  servo_ElevatorMotor.attach(ci_Elevator); // Elevator Motor

  //Set up encoders for: Elevator, left drive motor, right motor
  encoder_Elevator.init(1 / 0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_Elevator.setReversed(false);
  encoder_LeftMotor.init(1 / 0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(true);
  encoder_RightMotor.init(1 / 0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);

  //Control Pin
  pinMode(ci_HallEffect, INPUT);
  StarkTesseractClawOpen();
}

void loop() {
  Hall_Reading = analogRead(ci_HallEffect); // sets the reading Hall effect sensor
  //This is the actual RUNNING CODE//

  if (state_Var == 0) { //Drive along right wall
    PingBack();
    PingFront();
    //Sub-statements for maintaining optimal distance from wall
    if ((ul_Echo_TimeB - ul_Echo_TimeF > 40) && (ul_Echo_TimeB - ul_Echo_TimeF < 90)) {
      StarkGoStraight();
    }
    if (ul_Echo_TimeB - ul_Echo_TimeF < 40) {
      StarkTurnLeft();
    }
    if (ul_Echo_TimeB - ul_Echo_TimeF > 90) {
      StarkTurnRight();
    }
    if (abs(Hall_Reading - 514) > 15) { // If the tesseract is detected, go to next state
      state_Var++;
    }
  }

  if (state_Var == 1) { //Moving Elevator into position
    if (encoder_Elevator.getRawPosition() < 33) { //Drops elevator to a height so that the Tesseract Claw is aligned with it
      StarkElevatorDown();
    }
    if (encoder_Elevator.getRawPosition() >= 33) {
      AntiGrav(); // Holds the elevator in place
      StarkTesseractClawClose(); // grab the tesseract
      encoder_LeftMotor.zero();
      state_Var++;
    }
  }
  if (state_Var == 2) {
    if (encoder_LeftMotor.getRawPosition() < 70) { // Hardcode to get the robot off the wall
      StarkTurnRight();
    }
    else if (encoder_LeftMotor.getRawPosition() > 70 && encoder_LeftMotor.getRawPosition() < 120)
    {
      StarkGoStraight(); //Ensures the robot is sufficiently far from the wall so that when it sweeps
      //it doesn't collide with the wall again
    }
    else if (encoder_LeftMotor.getRawPosition() >= 120) {
      StarkSweepCW(); //Searching for pyramid
    }

    if (Serial.available() > 0) { // Checks if there's anything on the serial port
      if (Serial.read() == 69 || Serial.read() == 65) { // Checks if the IR sensors sees the letters "A" or "E" in ASCII
        state_Var++; // If "A" or "E" are seen, go to next state
      }
    }
  }

  if (state_Var == 3) {
    PingPyramid();
    StarkGoStraight();
    if (Serial.available() > 0) {                         //If the IR no longer sees the pyramid
      if (Serial.read() != 69 && Serial.read() != 65) {   //the Robot goes back to the state
        state_Var = 2;                                    //where is searches for the pyramid
      }
    }
    if (ul_Echo_TimePyramid / 58 == 7) { //Checks if the pyramid is in front of the
      StarkPyramidClawClose() ; //grabs the pyramid
      StarkTesseractClawOpen(); //drops the tesseract
      encoder_RightMotor.zero();
      encoder_Elevator.zero();
      state_Var++;
    }
  }
  if (state_Var == 4) {
    if (encoder_Elevator.getRawPosition() < 100) {
      StarkElevatorUp(); //moves the elevator up
    }
    if (encoder_Elevator.getRawPosition() >= 100) {
      state_Var++;
    }
  }
  if (state_Var == 5) { //HARDCODED SECTION FOR DROPPING THE PYRAMID OVER THE TESSERACT
    if (encoder_RightMotor.getRawPosition() < 138) {
      StarkSweepCCW(); 
    }
    if (encoder_RightMotor.getRawPosition() >= 138) {
      StarkPyramidClawOpen();
    }
  }

  //Setting servos write values and speeds
  servo_RightDriveMotor.writeMicroseconds(ul_Right_Motor_Speed);
  servo_LeftDriveMotor.writeMicroseconds(ul_Left_Motor_Speed);
  servo_ElevatorMotor.writeMicroseconds(ul_Elevator_Motor_Speed);
  servo_TesseractClaw.write(servo_Pos_Tess);
  servo_PyramidClaw.write(servo_Pos_Pyra);
}

void PingFront() //ping for front Ultrasonic
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_PingF, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_PingF, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_TimeF = pulseIn(ci_Ultrasonic_DataF, HIGH, 10000);
}

void PingPyramid() //ping for pyramid Ultrasonic
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_PingPyra, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_PingPyra, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_TimePyramid = pulseIn(ci_Ultrasonic_DataPyramid, HIGH, 10000);
}

void PingBack() //ping for back Ultrasonic
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_PingB, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_PingB, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_TimeB = pulseIn(ci_Ultrasonic_DataB, HIGH, 10000);
#ifdef DEBUG_ULTRASONIC_BACK
  Serial.print("Time(microseconds):");
  Serial.print(ul_Echo_TimeB, DEC);
  Serial.print (", cm:");
  Serial.println(ul_Echo_TimeB / 58);
#endif
}


void StarkGoStraight() { // Robot drives straight
  ul_Right_Motor_Speed = 1700;
  ul_Left_Motor_Speed = 1700;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkTurnRight() { // Robot turns right
  ul_Right_Motor_Speed = 1600;
  ul_Left_Motor_Speed = 1900;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkTurnLeft() {  //  Robot turns left
  ul_Right_Motor_Speed = 1700;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkTesseractClawOpen() { // Opens tesseract claw
  servo_Pos_Tess = 70;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkTesseractClawClose() { // Closes tesseract claw
  servo_Pos_Tess = 10;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkPyramidClawOpen() { // Opens pyramid claw
  servo_Pos_Pyra = 10;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkPyramidClawClose() { // Close pyramid claw
  servo_Pos_Tess = 70;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkElevatorUp() // Raises the elevator 
{
  ul_Elevator_Motor_Speed = 1400;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
}
void StarkElevatorDown() // Lowers the elevator 
{
  ul_Elevator_Motor_Speed = 1600;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
}
void StarkElevatorStop() { // Makes the elevator stop
  ul_Elevator_Motor_Speed = 1500;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
}
void AntiGrav() { // Holds the elevator up against the force of gravity
  ul_Elevator_Motor_Speed = 1450;
  // ul_Right_Motor_Speed = 1500;
  // ul_Left_Motor_Speed = 1500;
}
void StarkSweepCW() { // Spot rotation of robot in the clockwise direction
  ul_Right_Motor_Speed = 1300;
  ul_Left_Motor_Speed = 1700;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkSweepCCW() { // Spot rotation of robot in the counter clockwise direction
  ul_Right_Motor_Speed = 1700;
  ul_Left_Motor_Speed = 1300;
  ul_Elevator_Motor_Speed = 1450;
}

