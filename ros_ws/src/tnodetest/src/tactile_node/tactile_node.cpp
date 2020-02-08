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
#include <tactile_node/i2c.h>
#include <tactile_node/tactile_node_core.h>
#include <iostream>

#define RATE	10

int main(int argc,char **argv){
	// Initialize node
	ros::init(argc, argv, "tactile_node");

	// Set up I2C bus and pointer
	i2c *ptr;
	i2c bus;
	ptr = &bus;

	// Instantiate publisher object
	tactile_node tactileNode(ptr);

	// Set loop rate 
	ros::Rate loop_rate(RATE);
	// Main loop
	while(ros::ok()){
		// Publish tactile data
		tactileNode.publish_state();

		// Allow for callbacks
		ros::spinOnce();

		// Sleep for rest of cycle
		loop_rate.sleep();
	}
}