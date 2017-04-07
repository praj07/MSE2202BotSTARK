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
const int ci_Ultrasonic_PingB = 2 ; //input plug back US
const int ci_Ultrasonic_DataB = 3 ; //output plug back US
const int ci_Ultrasonic_PingF = 4 ; //input plug front US
const int ci_Ultrasonic_DataF = 5 ; //output plug front Us
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

long l_Elevator_Motor_Position;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

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
}
void loop() {
  PingBack();
  PingFront();
  delay(100);
  Serial.println(ul_Echo_TimeB- ul_Echo_TimeF);
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
  //#ifdef DEBUG_ULTRASONIC_FRONT
  //Serial.print("Time(microseconds):");
  //Serial.print(ul_Echo_TimeF,DEC);
  //Serial.print (", cm:");
  //Serial.println(ul_Echo_TimeF/58);
  //#endif
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
//#ifdef DEBUG_ULTRASONIC_BACK
//Serial.print("Time(microseconds):");
//Serial.print(ul_Echo_TimeB,DEC);
//Serial.print (", cm:");
//Serial.println(ul_Echo_TimeB/58);
//#endif
}


/*void StarkGoStraight() {
  servo_RightDriveMotor.writeMicroseconds(1900);
  servo_LeftDriveMotor.writeMicroseconds(1900);
  }
  void StarkTurnRight() {
  servo_RightDriveMotor.writeMicroseconds(1500);
  servo_LeftDriveMotor.writeMicroseconds(1900);
  }
  void StarkTurnLeft() {
  servo_RightDriveMotor.writeMicroseconds(1900);
  servo_LeftDriveMotor.writeMicroseconds(1500);
  }*/


