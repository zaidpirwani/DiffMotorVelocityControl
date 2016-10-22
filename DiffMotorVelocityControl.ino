// SERIAL COMMANDS
// SPEED: Abruptly set speed of motors, control with PID, speed sensed via encoders @ 100Hz (10ms)
// COMMAND:  D,speed_motor_left,speed_motor_right\n
// RESPONSE: d\n
// params: +/-SPEED - sign is direction, 0 is STOP, MIN and MAX SPEED is ???

// PID: Set PID params of either motor, for velocity control
// COMMAND:  H,Kp,Ki,Kd,1/2\n
// RESPONSE: h\n
// params: Kp,Ki,Kd are floats, 1/2 is motor identifier, 1==left

// PWM: Abruptly set speed of motors, OPEN LOOP
// COMMAND:  L,speed_motor_left,speed_motor_right\n
// RESPONSE: l\n
// params: +/-PWM - sign is direction, 0 is STOP, MIN and MAX SPEED is ???

// READ ENCODER: get encoder counter values
// COMMAND:  R\n
// RESPONSE: r,left_motor,right_motor\n
// params: signed 32-bit data count

// RESET ENCODER: reset encoder counter values
// COMMAND:  I\n
// RESPONSE: i\n
// params: reset both encoder counters to zero

// first param is of left motor, left motor and associated objects are numbered 1, right is 2

#include <math.h>
#include <EEPROM.h>
#include <Encoder.h>

Encoder enc1(2, 4); 
Encoder enc2(3, 5); 

#include "MusafirMotor.h"
MusafirMotor motor2(13, 12, 10);
MusafirMotor motor1(7, 6, 9);


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
unsigned long prevSpeedCheck = 0;
const long speedInterval = 10;    //10ms

#define DEBUG 0
#define MAGICADDRESS 42         // randomly defined eeprom 42 address

struct motorParams {
  int minPWM;
  int maxPWM;
  double kp;
  double ki;
  double kd;
};
motorParams motorParam1;
motorParams motorParam2;
long encPrev1 = 0, encPrev2 = 0;
long encDiff1 = 0, encDiff2 = 0;
int vel1 = 0, vel2 = 0;

const int wheel_diameter = 67;    //milimeters
const float wheel_radius = wheel_diameter/2.0;
const float circumference = (float)wheel_radius*M_PI;   //S=pi*r


boolean pidActive= false;
float error1 = 0, sum_error1 = 0, last_error1 = 0; 
float error2 = 0, sum_error2 = 0, last_error2 = 0; 
float pid1 = 0;  //  pid computation result
float pid2 = 0;


void setup() 
{

  inputString.reserve(100);
  Serial.begin(115200);
  enc1.write(0);
  enc2.write(0);
  motor1.setDir(FORWARD);
  motor2.setDir(FORWARD);

  int checkEEPROM=0;
  EEPROM.get(0, checkEEPROM);
  if(checkEEPROM==MAGICADDRESS){
    if(DEBUG) Serial.println("Reading from EEPROM.");
    EEPROM.get((const int)MAGICADDRESS, motorParam1);   //(address, data)
    EEPROM.get((const int)(MAGICADDRESS+sizeof(motorParams)), motorParam2);
  }
  else{
    //set default values
    if(DEBUG) Serial.println("Setting Default Values.");
    EEPROM.put(0, MAGICADDRESS);
    motorParam1.minPWM = 10;
    motorParam1.maxPWM = 225;
    motorParam1.kp = 10.0;
    motorParam1.ki = 1.0;
    motorParam1.kd = 0.0;
    EEPROM.put((const int)MAGICADDRESS, motorParam1);

    motorParam2.minPWM = 10;
    motorParam2.maxPWM = 225;
    motorParam2.kp = 10.0;
    motorParam2.ki = 1.0;
    motorParam2.kd = 0.0;
    EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorParam2);
  }

  motor1.setMinPWM(motorParam1.minPWM);
  motor1.setMaxPWM(motorParam1.maxPWM);

  motor2.setMinPWM(motorParam2.minPWM);
  motor2.setMaxPWM(motorParam2.maxPWM);

  resetEncoders();
  if(DEBUG){
    Serial.println("motorParam1");                                         
    Serial.print("minPWM ");
    Serial.println(motorParam1.minPWM);
    Serial.print("maxPWM ");
    Serial.println(motorParam1.maxPWM);
    Serial.print("kp ");
    Serial.println(motorParam1.kp);
    Serial.print("ki ");
    Serial.println(motorParam1.ki);
    Serial.print("kd ");
    Serial.println(motorParam1.kd);    
                                        
    Serial.println("motorParam2");
    Serial.print("minPWM ");
    Serial.println(motorParam2.minPWM);
    Serial.print("maxPWM ");
    Serial.println(motorParam2.maxPWM);
    Serial.print("kp ");
    Serial.println(motorParam2.kp);
    Serial.print("ki ");
    Serial.println(motorParam2.ki);
    Serial.print("kd ");
    Serial.println(motorParam2.kd);  
    Serial.println("-----X--------X--------X--------X-----");
  }
}

void loop() {
  if (stringComplete) {
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
        if(val1<0) { motor1.setDir(BACKWARD); val1 = -val1; }
        else         motor1.setDir(FORWARD);
        if(val2<0) { motor2.setDir(BACKWARD); val2 = -val2; }
        else         motor2.setDir(FORWARD);
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
        double p,i,d;
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
          motorParam1.kp = p;
          motorParam1.ki = i;
          motorParam1.kd = d;
          EEPROM.put((const int)MAGICADDRESS, motorParam1);
          if(DEBUG) Serial.println("MotorParam1 ");
        }
        else if(val1==2){
          motorParam2.kp = p;
          motorParam2.ki = i;
          motorParam2.kd = d;
          EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorParam2);
          if(DEBUG) Serial.println("MotorParam2 ");
        }
       if(DEBUG)
       { 
        Serial.print("kp: ");
        Serial.println(p);
        Serial.print("ki: ");
        Serial.println(i);
        Serial.print("kd: ");
        Serial.println(d);
       }
       Serial.println('h');
        break;
      case 'L':
        // COMMAND:  L,speed_motor_left,speed_motor_right\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1,c2).toInt();
        c1 = inputString.indexOf(',',c2)+1;
        val2 = inputString.substring(c1).toInt();
        if(val1<0) { motor1.setDir(BACKWARD); val1 = -val1; }
        else         motor1.setDir(FORWARD);
        if(val2<0) { motor2.setDir(BACKWARD); val2 = -val2; }
        else         motor2.setDir(FORWARD);
        pidActive= false;
        motor1.setPWM(val1);
        motor2.setPWM(val2);
        if(DEBUG){
          Serial.print("PWM1: "); Serial.println(val1);        
          Serial.print("PWM2: "); Serial.println(val2);
        }
        Serial.println('l');
        break;
      case 'R':
        // COMMAND:  R\n
        Serial.print("r,"+(String)enc1.read()+","+(String)enc2.read());
        //Serial.print(enc1.read());
        //Serial.print(',');
        //Serial.print(enc2.read());
        Serial.println();
        break;
      case 'I':
        resetEncoders();
        Serial.println('i');
        break;
      case 'M':
        //COMMAND M,maxPWM,minPWM,leftmotor/rightMotor \n
        int max_pwm,min_pwm;
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        max_pwm= inputString.substring(c1,c2).toInt();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        min_pwm= inputString.substring(c1,c2).toInt();
        c1=c2+1;
        val1= inputString.substring(c1).toInt();
        if(val1==1){
          motorParam1.maxPWM =max_pwm;
          motorParam1.minPWM =min_pwm;
          EEPROM.put((const int)MAGICADDRESS, motorParam1);
          if(DEBUG) Serial.println("MotorParam1 ");
        }
        else if(val1==2)
        {
          motorParam2.maxPWM =max_pwm;
          motorParam2.minPWM =min_pwm;
          EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorParam2);
          if(DEBUG) Serial.println("MotorParam2 ");
        }
        if(DEBUG){
          Serial.print("Max: ");
          Serial.println(max_pwm);
          Serial.print("Min: ");
          Serial.println(min_pwm);
        }
        Serial.println('m');
        break;
        case 'S':           // Send back the motor pid
        // COMMAND:  S,1/2\n
        int whichMotor;
        whichMotor=inputString.substring(2).toInt();
        if(whichMotor==1)
        {Serial.print("s,");
        Serial.print(motorParam1.kp);
        Serial.print(',');
        Serial.print(motorParam1.ki);
        Serial.print(',');
        Serial.print(motorParam1.kd);
        Serial.print(",1");
        Serial.println();
        }
        else if(whichMotor==2)
        {Serial.print("s,");
        Serial.print(motorParam2.kp);
        Serial.print(',');
        Serial.print(motorParam2.ki);
        Serial.print(',');
        Serial.print(motorParam2.kd);
        Serial.print(",2");
        Serial.println();
        }
        break;
    }
    inputString = "";
    stringComplete = false;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - prevSpeedCheck >= speedInterval) {
    prevSpeedCheck = currentMillis;
    int sp1,sp2;
    encDiff1=abs(enc1.read())-encPrev1;
    encDiff2=abs(enc2.read())-encPrev2;
    encPrev1=encPrev1+encDiff1;
    encPrev2=encPrev2+encDiff2;
    sp1=abs((float)encDiff1*9.58186); //(pi*r)/(1500*10ms) r= 91.5/2
    sp2=abs((float)encDiff2*9.58186);
    if(pidActive){
    error1=vel1-sp1;
    error2=vel2-sp2;
    sum_error1 += error1;
    sum_error2 += error2;
   // Serial.println(error2);
    if(error1>=0)
    { pid1 = (motorParam1.kp*error1)+ (motorParam1.ki*sum_error1)+ (motorParam1.kd*abs(error1-last_error1))/0.01;}
    else
    { pid1 = (motorParam1.kp*error1)+ (motorParam1.ki*sum_error1)- (motorParam1.kd*abs(error1-last_error1))/0.01;}
    
    if(error2>=0)
    {pid2 = (motorParam2.kp*error2)+ (motorParam2.ki*sum_error2)+ (motorParam2.kd*abs(error2-last_error2))/0.01; }
    else
    {pid2 = (motorParam2.kp*error2)+ (motorParam2.ki*sum_error2)- (motorParam2.kd*abs(error2-last_error2))/0.01; }
    //pid1 = (motorParam1.kp*error1)+ (motorParam1.ki*sum_error1)+ (motorParam1.kd*(error1-last_error1))/0.01;
    //pid2 = (motorParam2.kp*error2)+ (motorParam2.ki*sum_error2)+ (motorParam2.kd*(error2-last_error2))/0.01;
    error1=last_error1;
    error2=last_error2;
    pid1=constrain(pid1,motorParam1.minPWM,motorParam1.maxPWM);
    pid2=constrain(pid2,motorParam2.minPWM,motorParam2.maxPWM);
  // Serial.println(pid2);
    motor1.setPWM(pid1);
    motor2.setPWM(pid2);
    }      
 /* Serial.println("  ---  --- ");
    Serial.print("Velocity 2 ");  
    Serial.println(sp2);
    Serial.print("velocity 1 ");
    Serial.println(sp1);
  */

  }  

}
void serialEvent() 
{
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
void resetEncoders(void){
  enc1.write(0);
  enc2.write(0);
}


/*float velocity_time = 10;  //  time between velocity calculations
//  PID Parameters
float kp = 7.0, ki = 2.0, kd = 0.5;  //  tuning parameter
float error = 0, sum_error = 0, last_error = 0; 
float pid = 0;  //  pid computation result
int set_point = 25;  //  desired velocity in terms of cm/s
int dt = velocity_time;
float change_distance = 0;  //  do not alter this line
float new_time = 0, old_time = 0, change_time = 1;  //  do not alter this line
float velocity = 0;
*/



// enc1Diff = enc1.read() - enc1Diff;
//enc1Diff = enc1.read();    
// use formula on page 25 of khepera IV manual for speed calculation
//  velocity = get_velocity_pid();
//motor1.setPWM(pid);
//Serial.println(velocity);
// put code here for control of the second velocity as well.


/*float get_distance(void)  //  FUNCTION for calculating the Distance after last time...
{
  float ch_distance; 
  ch_distance = enc1.read();  //  reading encoder
  enc1.write(0);  //  seting encoder to zero again

  ch_distance /= 1500;  //  1500 encoder values is equal to 1 revolution 
  ch_distance *= circumference;  //  converting revolution to cm distance
  return ch_distance;
}

//  FUNCTION for calculating the velocity and PID result after certain time. eg 10ms
float get_velocity_pid(void)
{
  new_time = millis();
  if((new_time-old_time) >= velocity_time)
  {
    change_distance = get_distance();  //  get distance
    velocity = change_distance*1000;  //  1000 for miliseconds to seconds
    velocity /= (new_time-old_time);  //  devide by time is velocity
    old_time = new_time;
    //  GETTING compute its PID PID PID
    error = set_point-velocity;  //  calculating Error
    sum_error += error;  //  summation of error
    pid = kp*error+ki*sum_error+kd*(error-last_error)/dt;  //  PID result value(OUTPUT) 
    last_error = error;
    pid = constrain(pid,0,250);  //  defines limit
  }
  return velocity; 
}*/