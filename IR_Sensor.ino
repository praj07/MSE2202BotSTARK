int InputSig = 0;
void setup() {

  Serial.begin(2400);
}

void loop() {
  if (Serial.available () > 0) {
    InputSig = Serial.read();
    Serial.println(InputSig);
  }
}
