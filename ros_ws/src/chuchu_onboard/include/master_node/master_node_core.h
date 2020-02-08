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
#ifndef master_node_core_H_
#define master_node_core_H_

#include "ros/ros.h"
#include "chuchu_onboard/Int16ArrayHeader.h"
#include "std_srvs/SetBool.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <vector>

using namespace message_filters;

class master_node{
private:
	// Declare node handle
	ros::NodeHandle nh;

	// Declare publisher
	ros::Publisher pub;

	// Declare subscribers
	message_filters::Subscriber<chuchu_onboard::Int16ArrayHeader> motorSub;
	message_filters::Subscriber<chuchu_onboard::Int16ArrayHeader> tactileSub;

	// Declare initialize service
	ros::ServiceServer initSrv;

	// Initialize service callback
	bool init_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

	// Create sync policy
	typedef sync_policies::ApproximateTime
		<chuchu_onboard::Int16ArrayHeader, chuchu_onboard::Int16ArrayHeader> 
		MySyncPolicy;

	// Declare synchronizer
	Synchronizer<MySyncPolicy> sync;

	// Synchronizer callback
	void state_callback(const chuchu_onboard::Int16ArrayHeader::ConstPtr& motorData,
			  	  		const chuchu_onboard::Int16ArrayHeader::ConstPtr& tactileData);

public:
	// Initializer
	master_node();

	// If initialize signal is received, set to TRUE
	bool ready;
};


#endif