#include <Servo.h>
#include <I2CEncoder.h>
#define DEBUG_ULTRASONIC
Servo servo_ArmMotor;
Servo servo_Elevator;
I2CEncoder encoder_elevator;
const int ci_Arm_Motor = 11;
const int ci_Ultrasonic_Ping = 4;
const int ci_Ultrasonic_Data = 5;
const int ci_Elevator_Motor =6;
unsigned long ul_Echo_Time = 0;
int height = 0;

void setup()
{
  Serial.begin(9600);
  pinMode (ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
 // pinMode (ci_elevator_Motor, OUTPUT);
 // servo_Elevator.attach(ci_elevator_Motor);
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);
}
void loop (){
    servo_ArmMotor.write(120);
  delay(500);
  servo_ArmMotor.write(20);
  delay(500);
  //Serial.println(ul_Echo_Time);
}

void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time(microseconds):");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print (", cm:");
  Serial.println(ul_Echo_Time / 58);
#endif
}

