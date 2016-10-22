#include <Encoder.h>
Encoder myEnc1(2,4);
Encoder myEnc2(3,5);

#define PWMa 10
#define IN1a 13
#define IN2a 12

#define PWMb 9
#define IN1b 6
#define IN2b 7

const float wheel_radius = 3.35; // in cm
const float circumference = 2 * M_PI * wheel_radius;
const float tickDistance = (float)circumference/1500.0;

unsigned long previousMillis = 0;
const long interval = 100;

void setup() {
Serial.begin(115200);

pinMode(PWMa,OUTPUT);
pinMode(IN1a,OUTPUT);
pinMode(IN2a,OUTPUT);

pinMode(PWMb,OUTPUT);
pinMode(IN1b,OUTPUT);
pinMode(IN2b,OUTPUT);

myEnc1.write(0);
myEnc2.write(0);

digitalWrite(IN1a,LOW);
digitalWrite(IN2a,HIGH);
analogWrite(PWMa,200);
digitalWrite(IN1b,LOW);
digitalWrite(IN2b,HIGH);
analogWrite(PWMb,0);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    long encCurr1 = myEnc1.read();
    long encCurr2 = myEnc2.read();
    myEnc1.write(0);
    myEnc2.write(0);

    float distance1 = (float)encCurr1*tickDistance;
    float velocity1 = (float)distance1*(1000.0/interval);
    float distance2 = (float)encCurr2*tickDistance;
    float velocity2 = (float)distance2*(1000.0/interval);
    Serial.print(encCurr1);
    Serial.print(", ");
    Serial.print(distance1);
    Serial.print(", ");
    Serial.print(velocity1);
    Serial.print(" - ");
    Serial.print(encCurr2);
    Serial.print(", ");
    Serial.print(distance2);
    Serial.print(", ");
    Serial.println(velocity2);  
  }
}
