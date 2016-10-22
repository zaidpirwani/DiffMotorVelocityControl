// ---------------------------------------------------------------------------
// Musafir Motor LOW Level Control Library - v1 - 09/06/2016
//
// AUTHOR/LICENSE:
// Created by Zaid Pirwani - zaidpirwani@ejaadtech.com
// BASE CODE COPIED FROM NEWPING LIBRARY: https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
// Copyright 2015 License: GNU GPL v3 http://www.gnu.org/licenses/gpl.html
//
// LINKS:
// Project home: TODO
// Blog: TODO
//
// DISCLAIMER:
// This software is furnished "as is", without technical support, and with no 
// warranty, express or implied, as to its usefulness for any purpose.
//
// BACKGROUND:
// TODO
//
// FEATURES:
// * TODO
//
// CONSTRUCTOR:
//   MusafirMotor motor(channel_1_pin, channel_2_pin, pwm_pin)
//
// SYNTAX:
//   motor.setMinPWM(uint8_t minPWM)
//   uint8_t motor.getMinPWM()
//   motor.setMaxPWM(uint8_t maxPWM)
//   uint8_t motor.getMaxPWM()
//   motor.setPWM(uint8_t pwm)
//   uint8_t motor.getPWM()
//   motor.setDir(uint8_t dir)
//   uint8_t motor.getDir()
//
// 09/06/2016 v1.0 - Initial release.
// ---------------------------------------------------------------------------

#ifndef MusafirMotor_h
#define MusafirMotor_h

#if defined (ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
	#include <pins_arduino.h>
#endif

#if defined (__AVR__)
	#include <avr/io.h>
#endif

#define FORWARD   1
#define BACKWARD  2
#define BRAKE     3
#define RELEASE   4

class MusafirMotor {
	public:
		MusafirMotor(uint8_t ch1_pin, uint8_t ch2_pin, uint8_t pwm_pin);
		void setMinPWM(uint8_t minPWM);
		unsigned int getMinPWM();
		void setMaxPWM(uint8_t maxPWM);
		unsigned int getMaxPWM();
		void setPWM(uint8_t pwm);
		unsigned int getPWM();
		void setDir(uint8_t dir);
		unsigned int getDir();
	private:
		uint8_t _ch1_pin;
		uint8_t _ch2_pin;
		uint8_t _pwm_pin;
		uint8_t _pwm;
		uint8_t _dir;
		uint8_t _minPWM;
		uint8_t _maxPWM;
};

#endif
