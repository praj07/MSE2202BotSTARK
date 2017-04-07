#include <Servo.h>
#include <I2CEncoder.h>
Servo servo_SmallClaw;
Servo servo_Elevator;
I2CEncoder encoder_elevator;
const int ci_Small_Claw = 11;
//const int ci_elevator_Motor = 11;
int height = 0;
void setup()
{
  pinMode (ci_Small_Claw, OUTPUT);
  servo_SmallClaw.attach(ci_Small_Claw);
 // pinMode (ci_elevator_Motor, OUTPUT);
//  servo_Elevator.attach(ci_elevator_Motor);
}
void loop ()
{
  servo_SmallClaw.write(150);
  delay(1500);
  servo_SmallClaw.write(50);
  delay(1500);
}

