#include <Encoder.h>
#define PWM_R 10
#define IN1_R 13
#define IN2_R 12
#define PWM_L 9
#define IN1_L 7
#define IN2_L 6
Encoder myEnc_R(4, 2);
Encoder myEnc_L(3, 5);
float total_R, total_L, total_RL;
//total distance covered of right and left and sum of both
//......RIGHT MOTOR.......//
float distance_R = 1 ;
//DISTANCE MEASUREMENT 1=cm, 100=meters, 1000=km
float radius_R = 3.35/distance_R ;
//diameter in terms of "cm" (Put in cm always)
float circumference_R = 3.142 * radius_R * distance_R ;
float velocity_time_R = 10 ;
//10ms
float change_distance_R = 0 ;
//do not alter this line
float new_time_R = 0, old_time_R = 0, change_time_R = 1 ;
//do not alter this line
float velocity_R = 0 ;
//......LEFT MOTOR.......//
float distance_L = 1 ;
float radius_L = 3.35/distance_L ;
float circumference_L = 3.142 * radius_L * distance_L ;
float velocity_time_L = 10 ;
float change_distance_L = 0 ;
float new_time_L = 0, old_time_L = 0, change_time_L = 1 ;
float velocity_L = 0 ;
//......RIGHT MOTOR PID Parameters......//
float kp_R = 13.0, ki_R = 0.0, kd_R = 0.55;
//  tuning parameter
float error_R = 0, sum_error_R = 0, last_error_R = 0;
float pid_R = 0;
//  pid computation result
int set_point_R = 15;
//  desired velocity in terms of cm/s
int dt_R = velocity_time_R;
//......LEFT MOTOR PID Parameters......//
float kp_L = 14.6, ki_L = 0.0, kd_L = 0.6;
//  tuning parameter
float error_L = 0, sum_error_L = 0, last_error_L = 0;
float pid_L = 0;
//  pid computation result
int set_point_L = 18.3;
//  desired velocity in terms of cm/s
int dt_L = velocity_time_L;
void setup() {
	Serial.begin(115200);
	pinMode(PWM_R,OUTPUT);
	pinMode(IN1_R,OUTPUT);
	pinMode(IN2_R,OUTPUT);
	pinMode(PWM_L,OUTPUT);
	pinMode(IN1_L,OUTPUT);
	pinMode(IN2_L,OUTPUT);
	myEnc_R.write(0);
	myEnc_L.write(0);
}
void loop() {
	// if (total_R <=60){
	velocity_R = get_velocity_R();
	//Get velocity
	Right_Forward();
	//Motor direction 
	analogWrite(PWM_R, pid_R);
	//  }
	//  else{analogWrite(PWM_R, 0);
	//       digitalWrite(IN1_R, HIGH);
	//       digitalWrite(IN2_R, HIGH);
	//  }
	//  if (total_L <=60){
	velocity_L = get_velocity_L();
	Left_Forward();
	analogWrite(PWM_L, pid_L);
	// }
	//    else{analogWrite(PWM_L, 0);
	//       digitalWrite(IN1_L, HIGH);
	//       digitalWrite(IN2_L, HIGH);
	//    }
	total_RL = (total_R + total_L)/2;
	/*
float total_distance = (velocity_R + velocity_L)/2;
float average_velocity = total_distance/10;
float theta = (velocity_R - velocity_L)/16; //16cm dist between tyres
 right = average_velocity*sin(theta);
 left = average_velocity*cos(theta);
*/
	Readings();
	//Serial values
}
void Right_Forward(void) {
	digitalWrite(IN1_R,LOW);
	digitalWrite(IN2_R,HIGH);
}
void Right_Backward(void) {
	digitalWrite(IN1_R,HIGH);
	digitalWrite(IN2_R,LOW);
}
void Left_Forward(void) {
	digitalWrite(IN1_L,HIGH);
	digitalWrite(IN2_L,LOW);
}
void Left_Backward(void) {
	digitalWrite(IN1_L,LOW);
	digitalWrite(IN2_L,HIGH);
}
//  FUNCTION for calculating the velocity and PID result after certain time. eg 10ms
float get_velocity_R(void) {
	new_time_R = millis();
	if((new_time_R - old_time_R) >= velocity_time_R) {
		change_distance_R = get_distance_R();
		total_R += change_distance_R;
		velocity_R = change_distance_R * 1000;
		velocity_R/= (new_time_R - old_time_R);
		old_time_R = new_time_R;
		error_R = set_point_R-velocity_R;
		//  calculating Error
		sum_error_R += error_R;
		//  summation of error
		pid_R = kp_R*error_R + ki_R*sum_error_R + kd_R*(error_R-last_error_R)/dt_R;
		//  PID result value(OUTPUT)
		last_error_R = error_R;
		pid_R = constrain(pid_R,40,255);
		//  defines limit
	}
	return velocity_R;
}
float get_distance_R(void) {
	float change_distance_R1 = myEnc_R.read();
	myEnc_R.write(0);
	change_distance_R1/= 1500;
	change_distance_R1*= circumference_R;
	//converting revolution to cm
	return change_distance_R1;
}
float get_velocity_L(void) {
	new_time_L = millis();
	if((new_time_L - old_time_L) >= velocity_time_L) {
		change_distance_L = get_distance_L();
		total_L += change_distance_L;
		velocity_L = change_distance_L * 1000;
		velocity_L/= (new_time_L - old_time_L);
		old_time_L = new_time_L;
		error_L = set_point_L-velocity_L;
		//  calculating Error
		sum_error_L += error_L;
		//  summation of error
		pid_L = kp_L*error_L + ki_L*sum_error_L + kd_L*(error_L-last_error_L)/dt_L;
		//  PID result value(OUTPUT)
		last_error_L = error_L;
		pid_L = constrain(pid_L,40,255);
		//  defines limit
	}
	return velocity_L;
}
float get_distance_L(void) {
	float change_distance_L1 = myEnc_L.read();
	myEnc_L.write(0);
	change_distance_L1/= 1500;
	change_distance_L1*= circumference_L;
	//converting revolution to cm
	return change_distance_L1;
}
void Readings() {
	Serial.print(velocity_R);
	Serial.print('\t');
	Serial.print(velocity_L);
	Serial.print('\t');
	Serial.print(total_R);
	Serial.print('\t');
	Serial.print(total_L);
	Serial.print('\t');
	Serial.println(total_RL);
}