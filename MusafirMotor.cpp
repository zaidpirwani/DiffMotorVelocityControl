// ---------------------------------------------------------------------------
// Created by Zaid Pirwani - teckel@leethost.com
// Copyright 2015 License: GNU GPL v3 http://www.gnu.org/licenses/gpl.html
//
// See "MusafirMotor.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------

#include "MusafirMotor.h"
	

// ---------------------------------------------------------------------------
// MusafirMotor constructor
// ---------------------------------------------------------------------------

MusafirMotor::MusafirMotor(uint8_t ch1_pin, uint8_t ch2_pin, uint8_t pwm_pin) {
	_ch1_pin = ch1_pin;
	_ch2_pin = ch2_pin;
	_pwm_pin = pwm_pin;
	pinMode(ch1_pin, OUTPUT);
	pinMode(ch2_pin, OUTPUT);
	pinMode(pwm_pin, OUTPUT);
  setDir(FORWARD);
}


// ---------------------------------------------------------------------------
// Standard  methods
// ---------------------------------------------------------------------------

void MusafirMotor::setMinPWM(uint8_t minPWM) {
	_minPWM = minPWM;
}

unsigned int MusafirMotor::getMinPWM() {
	return(_minPWM);
}

void MusafirMotor::setMaxPWM(uint8_t maxPWM) {
	_maxPWM = maxPWM;
}

unsigned int MusafirMotor::getMaxPWM() {
	return(_maxPWM);
}

void MusafirMotor::setPWM(uint8_t pwm) {
	_pwm = constrain(pwm, _minPWM, _maxPWM);
	analogWrite(_pwm_pin,_pwm);
}

unsigned int MusafirMotor::getPWM() {
	return(_pwm);
}

void MusafirMotor::setDir(uint8_t dir) {
	if(_dir!=dir){
		_dir = dir;		
		switch(_dir){
			case FORWARD:
				digitalWrite(_ch1_pin, HIGH);
				digitalWrite(_ch2_pin, LOW);
				break;
			case BACKWARD:
				digitalWrite(_ch1_pin, LOW);
				digitalWrite(_ch2_pin, HIGH);
				break;
			case BRAKE:
				digitalWrite(_ch1_pin, HIGH);
				digitalWrite(_ch2_pin, HIGH);
				break;
			case RELEASE:
				digitalWrite(_ch1_pin, LOW);
				digitalWrite(_ch2_pin, LOW);
				break;
		}
	}
}

unsigned int MusafirMotor::getDir() {
	return(_dir);
}
