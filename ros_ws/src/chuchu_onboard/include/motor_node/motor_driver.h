/*
 *      ___           ___           ___           ___           ___           ___
 *     /\  \         /\__\         /\__\         /\  \         /\__\         /\__\
 *    /::\  \       /:/  /        /:/  /        /::\  \       /:/  /        /:/  /
 *   /:/\:\  \     /:/__/        /:/  /        /:/\:\  \     /:/__/        /:/  /
 *  /:/  \:\  \   /::\  \ ___   /:/  /  ___   /:/  \:\  \   /::\  \ ___   /:/  /  ___
 * /:/__/ \:\__\ /:/\:\  /\__\ /:/__/  /\__\ /:/__/ \:\__\ /:/\:\  /\__\ /:/__/  /\__\
 * \:\  \  \/__/ \/__\:\/:/  / \:\  \ /:/  / \:\  \  \/__/ \/__\:\/:/  / \:\  \ /:/  /
 *  \:\  \            \::/  /   \:\  /:/  /   \:\  \            \::/  /   \:\  /:/  /
 *   \:\  \           /:/  /     \:\/:/  /     \:\  \           /:/  /     \:\/:/  /
 *    \:\__\         /:/  /       \::/  /       \:\__\         /:/  /       \::/  /
 *     \/__/         \/__/         \/__/         \/__/         \/__/         \/__/
 */
// Authors(s): Grant Rudd and Jesse Cushing
// ENGIN 492 Spring 2018	 
// Revison date: 3/1/18
#pragma once
#ifndef Motor_HPP_
#define Motor_HPP_

#include "ChuPIO.h"
#include <string>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include "pid.h"
using namespace std;
//BBB pins (hardware)
#define APin "P9_40" 		//All analog reads take place on P9_40
#define Mux_A "P9_31"		//LSB of mux select
#define Mux_B "P9_30"
#define Mux_C "P9_29"		//MSB of mux select

#define STOP_THRESHOLD	1
#define MINIMUM_PWM		20
#define MAX_POSITION	90
#define MIN_POSITION	10

class Motor {
private:
	std::string Dir1, Dir2, PWMPin, MuxEnablePin;
	int PosMuxSelect, CurrentMuxSelect;
	PID pid;

	// Getters for position and current
	float get_position(void);
	float get_current(void);

public:
	//Constructor Inputs:
	//DIRECTION 1 :: DIRECTION 2 :: PWM Pin :: Mux Enable Pin :: PID Parameters (Kp, Ki, Kd) :: Position Mux :: Current Mux
	Motor(std::string Dir1, std::string Dir2, std::string PWMPin,
			std::string MuxEnablePin, int PosMuxSelect, int CurrentMuxSelect);
	// Class Functions
	void run_position();
	void stop(void);
	void set_setpoint(float setPoint);
	float get_setpoint();
	void set_pid(float p, float i, float d);

	float currentPosition, currentCurrent, lastPosition;
	bool atGoal;
};

#endif /* Motor_HPP_ */
