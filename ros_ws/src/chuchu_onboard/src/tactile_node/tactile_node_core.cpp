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
#include<tactile_node/tactile_node_core.h>

// Initialize node
tactile_node::tactile_node(i2c* _ptr) : thumb(_ptr, 0),
										finger1(_ptr, 2),
										finger2(_ptr, 3),
										finger3(_ptr, 1)
{	// Set up publisher
	tpub = nh.advertise<chuchu_onboard::Int16ArrayHeader>("tactile_states", 100);

	// Set up calibration service
	calibSrv = nh.advertiseService("calibrate_sensors", &tactile_node::calibrate_all, this);
}

// Publisher function
void tactile_node::publish_state(){
	// Publish binned state of tactile sensors and maximum value

	// Create array for tactile sensors
	chuchu_onboard::Int16ArrayHeader msg;

	// Update sensor values
	thumb.update_pressures();
	finger1.update_pressures();
	finger2.update_pressures();
	finger3.update_pressures();

	// Concatenate pressure vectors into msg
	msg.data.insert(msg.data.end(),
					thumb.sensorVals.begin(),
					thumb.sensorVals.end());
	msg.data.insert(msg.data.end(),
					finger1.sensorVals.begin(),
					finger1.sensorVals.end());
	msg.data.insert(msg.data.end(),
					finger2.sensorVals.begin(),
					finger2.sensorVals.end());
	msg.data.insert(msg.data.end(),
					finger3.sensorVals.begin(),
					finger3.sensorVals.end());

	// Add time stamp
	msg.header.stamp = ros::Time::now();

    // Publish message
    tpub.publish(msg);
}

bool tactile_node::calibrate_all(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
	// Calibrate all sensor modules
	thumb.calibrate();
	finger1.calibrate();
	finger2.calibrate();
	finger3.calibrate();

	// Respond to request
	res.success = true;
	res.message = "Sensors Calibrated";

	return true;
}