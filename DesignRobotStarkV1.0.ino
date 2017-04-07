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
const int ci_Elevator = 6;
const int ci_Mode_Switch = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9 ;
const int ci_Pyramid_Claw = 10;
const int ci_Tesseract_Claw = 11;

//Analog Pins
const int ci_HallEffect = A0;
const int ci_I2C_SDA = A4;
const int ci_I2C_SCL = A5;



unsigned long ul_Echo_TimeF;
unsigned long ul_Echo_TimeB;
unsigned long ul_Echo_TimePyramid;

int Hall_Reading;
int servo_Pos_Tess;
int servo_Pos_Pyra;

long l_Elevator_Motor_Position;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_Left_Motor_Speed;
unsigned long ul_Right_Motor_Speed;
unsigned long ul_Elevator_Motor_Speed;

int state_Var = 0;

bool runtime = false;

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

  //Drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightDriveMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftDriveMotor.attach(ci_Left_Motor);

  //Setup claws
  pinMode(ci_Tesseract_Claw, OUTPUT);
  servo_TesseractClaw.attach(ci_Tesseract_Claw);
  pinMode(ci_Pyramid_Claw, OUTPUT);
  servo_PyramidClaw.attach(ci_Pyramid_Claw);

  //Setup Elevator
  pinMode(ci_Elevator, OUTPUT);
  servo_ElevatorMotor.attach(ci_Elevator);

  //Set up encoders for: Elevator, left drive motor, right motor
  encoder_Elevator.init(1 / 0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_Elevator.setReversed(false);
  encoder_LeftMotor.init(1 / 0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(true);
  encoder_RightMotor.init(1 / 0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);

  //Control Pin
  pinMode(ci_Mode_Switch, INPUT);
  pinMode(ci_HallEffect, INPUT);
  StarkTesseractClawOpen();
}

void loop() {
  Hall_Reading = analogRead(ci_HallEffect);
  //servo_TesseractClaw.detach();
  //servo_PyramidClaw.detach();
  //servo_ElevatorMotor.detach();
  Serial.println(Hall_Reading - 514);

  //This is the actual RUNNING CODE//

  if (state_Var == 0) { //Drive along right wall
    // PingBack();
    //PingFront();
    //if ((ul_Echo_TimeB - ul_Echo_TimeF > 40) && (ul_Echo_TimeB - ul_Echo_TimeF < 90)) {
    // Serial.println("Go Straight");
    StarkGoStraight();
    AntiGrav();
    if (abs(Hall_Reading - 514) > 15) {
      state_Var++;
    }
  }
  if (state_Var == 1) { //Moving Elevator into position
    if (encoder_Elevator.getRawPosition() < 33) {
      StarkElevatorDown();
    }
    if (encoder_Elevator.getRawPosition() >= 33) {
      AntiGrav();
      StarkTesseractClawClose();
      encoder_LeftMotor.zero();
      state_Var++;
    }
  }
  if (state_Var == 2) {
    if (encoder_LeftMotor.getRawPosition() < 70) {
      StarkTurnRight();
    }
    else if (encoder_LeftMotor.getRawPosition() > 70 && encoder_LeftMotor.getRawPosition() < 120)
    {
      StarkGoStraight();
    }
    else if (encoder_LeftMotor.getRawPosition() >= 120)
    { StarkSweepCW();
    }

    if (Serial.available() > 0) {
      if (Serial.read() == 69 || Serial.read() == 65) {
        state_Var++;
      }
    }
  }

  if (state_Var == 3) {
    PingPyramid();
    StarkGoStraight();
    if (Serial.available() > 0) {
      if (Serial.read() != 69 && Serial.read() != 65) {
        state_Var = 2;
      }
    }
    if (ul_Echo_TimePyramid / 58 == 7) {
      StarkPyramidClawClose() ;
      StarkTesseractClawOpen();
      encoder_RightMotor.zero();
      encoder_Elevator.zero();
      state_Var++;
    }
  }
  if (state_Var == 4) {
    if (encoder_Elevator.getRawPosition() < 100) {
      StarkMotorUp()
    }
    if (encoder_Elevator.getRawPosition() >= 100){
      state_Var++;
    }
  }
if (state_Var == 5) {
    if (encoder_RightMotor.getRawPosition() < 138) {
      StarkSweepCCW();
    }
    if (encoder_RightMotor.getRawPosition() >= 138) {
      StarkPyramidClawOpen();
    }
  }

  servo_RightDriveMotor.writeMicroseconds(ul_Right_Motor_Speed);
  servo_LeftDriveMotor.writeMicroseconds(ul_Left_Motor_Speed);
  servo_ElevatorMotor.writeMicroseconds(ul_Elevator_Motor_Speed);
  servo_TesseractClaw.write(servo_Pos_Tess);
  servo_PyramidClaw.write(servo_Pos_Pyra);
}

void PingFront()
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

void PingPyramid()
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

void PingBack()
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


void StarkGoStraight() {
  ul_Right_Motor_Speed = 1700;
  ul_Left_Motor_Speed = 1700;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkTurnRight() {
  ul_Right_Motor_Speed = 1600;
  ul_Left_Motor_Speed = 1900;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkTurnLeft() {
  ul_Right_Motor_Speed = 1700;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkTesseractClawOpen() {
  servo_Pos_Tess = 70;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkTesseractClawClose() {
  servo_Pos_Tess = 10;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkPyramidClawOpen() {
  servo_Pos_Pyra = 10;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkPyramidClawClose() {
  servo_Pos_Tess = 70;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkElevatorUp()
{
  ul_Elevator_Motor_Speed = 1400;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
}
void StarkElevatorDown()
{
  ul_Elevator_Motor_Speed = 1600;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
}
void StarkElevatorStop() {
  ul_Elevator_Motor_Speed = 1500;
  ul_Right_Motor_Speed = 1500;
  ul_Left_Motor_Speed = 1500;
}
void AntiGrav() {
  ul_Elevator_Motor_Speed = 1450;
  // ul_Right_Motor_Speed = 1500;
  // ul_Left_Motor_Speed = 1500;
}
void StarkSweepCW() {
  ul_Right_Motor_Speed = 1300;
  ul_Left_Motor_Speed = 1700;
  ul_Elevator_Motor_Speed = 1450;
}
void StarkSweepCCW() {
  ul_Right_Motor_Speed = 1700;
  ul_Left_Motor_Speed = 1300;
  ul_Elevator_Motor_Speed = 1450;
}

