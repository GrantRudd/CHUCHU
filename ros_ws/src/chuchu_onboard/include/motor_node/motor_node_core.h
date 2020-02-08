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
#ifndef motor_node_core_H_
#define motor_node_core_H_

#include "ros/ros.h"
#include <motor_node/motor_driver.h>

#include "chuchu_onboard/Int16ArrayHeader.h"
#include "std_srvs/SetBool.h"

// Motor Parameters
#define Motor1_Direction1	"P8_30"
#define Motor1_Direction2	"P8_29"
#define Motor1_PWM			"P8_36"
#define Motor1_Mux			"P9_27"
#define Motor1_Position		5
#define Motor1_Current		3

#define Motor2_Direction1	"P8_27"
#define Motor2_Direction2	"P8_28"
#define Motor2_PWM			"P8_34"
#define Motor2_Mux			"P9_27"
#define Motor2_Position		7
#define Motor2_Current		0

#define Motor3_Direction1	"P9_23"
#define Motor3_Direction2	"P9_24"
#define Motor3_PWM			"P9_22"
#define Motor3_Mux			"P9_27"
#define Motor3_Position		6
#define Motor3_Current		1

#define Motor4_Direction1	"P9_26"
#define Motor4_Direction2	"P9_25"
#define Motor4_PWM			"P9_21"
#define Motor4_Mux			"P9_27"
#define Motor4_Position		4
#define Motor4_Current		2

#define Motor5_Direction1	"P8_33"
#define Motor5_Direction2	"P8_35"
#define Motor5_PWM			"P8_13"
#define Motor5_Mux			"P9_41"
#define Motor5_Position		5
#define Motor5_Current		3

#define Motor6_Direction1	"P8_31"
#define Motor6_Direction2	"P8_32"
#define Motor6_PWM			"P8_19"
#define Motor6_Mux			"P9_41"
#define Motor6_Position		7
#define Motor6_Current		0

#define Motor7_Direction1	"P9_15"
#define Motor7_Direction2	"P9_16"
#define Motor7_PWM			"P9_28"
#define Motor7_Mux			"P9_41"
#define Motor7_Position		6
#define Motor7_Current		1

#define Motor8_Direction1	"P9_18"
#define Motor8_Direction2	"P9_17"
#define Motor8_PWM			"P9_42"
#define Motor8_Mux			"P9_41"
#define Motor8_Position		4
#define Motor8_Current		2


class motor_node{
private:
	// Create node handle
	ros::NodeHandle nh;
	// Publisher attributes
	ros::Publisher mpub;
	// Subscriber attributes
	ros::Subscriber speedsub;

	// Callback for velocity callback
	void position_callback(const chuchu_onboard::Int16ArrayHeader& msg);

	// Motor Objects
	Motor motor1;
	Motor motor2;
	Motor motor3;
	Motor motor4;
	Motor motor5;
	Motor motor6;
	Motor motor7;
	Motor motor8;

	// Timers and callbacks
	ros::Timer timer10Hz;
	ros::Timer timer100Hz;
	void timer10HzCallback(const ros::TimerEvent& event);
	void timer100HzCallback(const ros::TimerEvent& event);

	// Open service
	ros::ServiceServer openSrv;

	// Open service callback
	bool open(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

public:
	// Constructor
	motor_node();
	// Publish motor states
	void publish_state();
	// Run the motors (updates current and position values)
	void run_all();
	// Update PID parameters
	void update_pid();
};

#endif