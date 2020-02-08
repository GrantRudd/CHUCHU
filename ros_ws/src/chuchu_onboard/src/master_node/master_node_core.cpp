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
#include "master_node/master_node_core.h"


master_node::master_node():motorSub(nh, "motor_states", 1),
						   tactileSub(nh, "tactile_states", 1),
						   sync(MySyncPolicy(10), motorSub, tactileSub)
{
	// Initialize publisher
	pub = nh.advertise<chuchu_onboard::Int16ArrayHeader>
						("chuchu_state", 5);

	// Initialize service server
	initSrv = nh.advertiseService("init_hardware", &master_node::init_callback, this);

	// Bind synchronizer to callback
	sync.registerCallback(boost::bind(&master_node::state_callback, this, _1, _2));

	// Initialize signal not received
	ready = false;
}

void master_node::state_callback(const chuchu_onboard::Int16ArrayHeader::ConstPtr& motorData,
			  			   		 const chuchu_onboard::Int16ArrayHeader::ConstPtr& tactileData)
{	
	// Create empty message
	chuchu_onboard::Int16ArrayHeader msg;

	// Insert motor states
	msg.data.insert(msg.data.end(),
					motorData->data.begin(),
					motorData->data.end());

	// Insert tactile states
	msg.data.insert(msg.data.end(),
					tactileData->data.begin(),
					tactileData->data.end());

	pub.publish(msg);
}

bool master_node::init_callback(std_srvs::SetBool::Request &req,
								std_srvs::SetBool::Response &res)
{	
	// Open hand
	// Create service client
	ros::ServiceClient openClient = nh.serviceClient<std_srvs::SetBool>("open_hand");

	// Create service object
	std_srvs::SetBool openSrv;
	openSrv.request.data = true;

	// Call service
	if(openClient.call(openSrv)){
		// Forward response from motor note
		res = openSrv.response;

		// set "ready" to true
		ready = true;
		
	}else{
		// In the case the motornode client returns false
		res.success = false;
		res.message = "Failed to communicate w/motors";
	}

	// Calibrate sensors
	// Create service client
	ros::ServiceClient calibrateClient = nh.serviceClient<std_srvs::SetBool>("calibrate_sensors");

	// Create service object
	std_srvs::SetBool calibrateSrv;
	calibrateSrv.request.data = true;

	// Call service
	calibrateClient.call(calibrateSrv);
	
	return true;
}