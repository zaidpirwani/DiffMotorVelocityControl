#define DEBUG 1
#define MAGICADDRESS 7
// randomly(or is it!) defined eeprom 42 address

// SET VELOCITY:  D,speed_motor_left,speed_motor_right\n
// SET PIDs:      H,P,I,D,1/2\n
// READ ENCODER:  R\n
// SET PWM:       L,speed_motor_left,speed_motor_right\n
// RESET ENCODER: I\n

#include <math.h>
#include <EEPROM.h>

#include <Encoder.h>
Encoder encL(2,4);
Encoder encR(3,5);

#include "MusafirMotor.h"
MusafirMotor motorL(13, 12, 10);
MusafirMotor motorR(6, 7, 9);
struct motorParams {
  double kp;
  double ki;
  double kd;
};
motorParams motorPIDL;
motorParams motorPIDR;
int vel1 = 0, vel2 = 0;
boolean pidActive= false;

const float wheel_radius = 3.35; // in cm
const float circumference = 2 * M_PI * wheel_radius;
const float tickDistance = (float)circumference/1500.0;

unsigned long previousMillis = 0;
const long interval = 100;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  Serial.begin(115200);
  inputString.reserve(100);
  encL.write(0);
  encR.write(0);
  motorL.setDir(FORWARD);
  motorR.setDir(FORWARD);

  initEEPROM();
}

void loop() {
  if (stringComplete) {
    interpretSerialData();
    stringComplete = false;
    inputString = "";
  }
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    long encCurr1 = encL.read();
    long encCurr2 = encR.read();
    resetEncoders();

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

void interpretSerialData(void){
    int c1=1, c2=1;
    int val1=0, val2=0;
    switch(inputString[0]){
      case 'D':
        // COMMAND:  D,speed_motor_left,speed_motor_right\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1,c2).toInt();
        c1 = c2+1;
        val2 = inputString.substring(c1).toInt();
        if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else         motorL.setDir(FORWARD);
        if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else         motorR.setDir(FORWARD);
        vel1 = val1;
        vel2 = val2;
        if(DEBUG){
          Serial.print("Velocity 1 ");
          Serial.println(vel1);
          Serial.print("Velocity 2 ");
          Serial.println(vel2);
        }         
        Serial.println('d');
        pidActive= true;
        break;
      case 'H':
        // COMMAND:  H,P,I,D,1/2\n
        float p,i,d;
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        p = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        i = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        d = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        val1 = inputString.substring(c1).toInt();
        if(val1==1) {
          motorPIDL.kp = p;
          motorPIDL.ki = i;
          motorPIDL.kd = d;
          EEPROM.put((const int)MAGICADDRESS, motorPIDL);
          if(DEBUG) Serial.println("motorPIDL ");
        }
        else if(val1==2){
          motorPIDR.kp = p;
          motorPIDR.ki = i;
          motorPIDR.kd = d;
          EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
          if(DEBUG) Serial.println("motorPIDR ");
        }
        Serial.print("kp: ");
        Serial.println(p);
        Serial.print("ki: ");
        Serial.println(i);
        Serial.print("kd: ");
        Serial.println(d);
        Serial.println('h');
        break;
      case 'L':
        // COMMAND:  L,speed_motor_left,speed_motor_right\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1,c2).toInt();
        c1 = inputString.indexOf(',',c2)+1;
        val2 = inputString.substring(c1).toInt();
        if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else         motorL.setDir(FORWARD);
        if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else         motorR.setDir(FORWARD);
        pidActive= false;
        motorL.setPWM(val1);
        motorR.setPWM(val2);
        if(DEBUG){
          Serial.print("PWM1: "); Serial.println(val1);        
          Serial.print("PWM2: "); Serial.println(val2);
        }
        Serial.println('l');
        break;
      case 'R':
        // COMMAND:  R\n
        Serial.print("r,");
        Serial.print(encL.read());
        Serial.print(',');
        Serial.print(encR.read());
        Serial.println();
        break;
      case 'I':
        // COMMAND: I\n
        resetEncoders();
        Serial.println('i');
        break;
      default:
        Serial.print("UNKNOWN COMMAND: ");
        Serial.println(inputString);
        break;
    }
}

void initEEPROM(void){
  int checkEEPROM=0;
  EEPROM.get(0, checkEEPROM);
  if(checkEEPROM==MAGICADDRESS){
    if(DEBUG) Serial.println("Reading from EEPROM.");
    EEPROM.get((const int)MAGICADDRESS, motorPIDL);   //(address, data)
    EEPROM.get((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
  }
  else{
    //set default values
    if(DEBUG) Serial.println("Setting Default Values.");
    EEPROM.put(0, MAGICADDRESS);
    motorPIDL.kp = 1.0;
    motorPIDL.ki = 0.0;
    motorPIDL.kd = 0.0;
    EEPROM.put((const int)MAGICADDRESS, motorPIDL);

    motorPIDR.kp = 1.0;
    motorPIDR.ki = 0.0;
    motorPIDR.kd = 0.0;
    EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
  }  
}

void resetEncoders(void){
  encL.write(0);
  encR.write(0);
}
