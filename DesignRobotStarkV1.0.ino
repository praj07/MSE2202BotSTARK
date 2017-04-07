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
const int ci_Ultrasonic_Ping = 2 ; //output Plug
const int ci_Ultrasonic_Data = 3 ; //input Plug
const int ci_Elevator = 6;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9 ;
const int ci_Pyramid_Claw = 10;
const int ci_Tesseract_Claw = 11;

const int ci_HallEffect = A0;
const int ci_I2C_SDA = A4;
const int ci_I2C_SCL = A5;


unsigned long ul_Echo_Time;

long l_Elevator_Motor_Position;

int runtime = 0;
int buttonState = 0; 

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  Serial.begin(2400);

  //Ultrasonics
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

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

  //Set up elevator encoder
  encoder_Elevator.init(1 / 0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_Elevator.setReversed(false);
  encoder_LeftMotor.init(1 / 0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(true);
  encoder_RightMotor.init(1 / 0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);

  pinMode(ci_Mode_Button, INPUT);

}

void loop() {

  buttonState = digitalRead(ci_Mode_Button);
  Serial.println(runtime);
  if (buttonState == LOW) {
    runtime++;
  }
  if (runtime % 2 == 1 ) {
    servo_RightDriveMotor.writeMicroseconds(1700);
    servo_LeftDriveMotor.writeMicroseconds(1700);
  }
  else {
    servo_RightDriveMotor.writeMicroseconds(1500);
    servo_LeftDriveMotor.writeMicroseconds(1500);
  }

}
