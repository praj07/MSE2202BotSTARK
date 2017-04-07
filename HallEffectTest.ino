const int hallEffectSensor = A0;
const int EM = 8;

void setup() {
  // put your setup code here, to run once:
  pinMode(hallEffectSensor,INPUT);
  pinMode(EM, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(hallEffectSensor)-514);
  delay(20);
}
