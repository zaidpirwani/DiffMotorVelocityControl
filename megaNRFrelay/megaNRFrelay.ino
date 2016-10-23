// Arduino Mega on Robot, with NRF and Serial3 to Nano with Motors
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
}

void loop() {
  if(Serial.available()>0){
    Serial3.write(Serial.read());
  }
  if(Serial3.available()>0){
    Serial.write(Serial3.read());
  }
}
