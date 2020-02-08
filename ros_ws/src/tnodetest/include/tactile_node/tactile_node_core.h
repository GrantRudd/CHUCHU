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
// Revison date: 3/14/18
#pragma once
#ifndef tactile_node_core_H_
#define tactile_node_core_H_

#include "ros/ros.h"
#include "chuchu_onboard/Int16ArrayHeader.h"

#include<tactile_node/tactile_sensor.h>
#include "std_srvs/SetBool.h"

class tactile_node{
private:
	ros::NodeHandle nh;
	ros::Publisher tpub;

	// I2C attributes
	i2c *ptr;
	i2c bus;

	// Tactile sensors 
	tactile_sensor	thumb;
	tactile_sensor	finger1;
	tactile_sensor	finger2; 
	tactile_sensor	finger3;

	// Calibration service
	ros::ServiceServer calibSrv;

	// Calibration callback
	bool calibrate_all(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

public:
	tactile_node(i2c* _ptr);
	void publish_state();
};

#endif