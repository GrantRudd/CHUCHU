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

#include <motor_node/motor_node_core.h>

// Constructor
motor_node::motor_node(): 	motor1 (Motor1_Direction1, Motor1_Direction2, 
								 	Motor1_PWM, Motor1_Mux, 
								 	Motor1_Position, Motor1_Current),

						 	motor2 (Motor2_Direction1, Motor2_Direction2, 
								 	Motor2_PWM, Motor2_Mux, 
								  	Motor2_Position, Motor2_Current),
						 	
						 	motor3 (Motor3_Direction1, Motor3_Direction2, 
								 	Motor3_PWM, Motor3_Mux, 
								  	Motor3_Position, Motor3_Current),
						 	
						 	motor4 (Motor4_Direction1, Motor4_Direction2, 
								 	Motor4_PWM, Motor4_Mux, 
								  	Motor4_Position, Motor4_Current),
						 	
						 	motor8 (Motor5_Direction1, Motor5_Direction2, 
								 	Motor5_PWM, Motor5_Mux, 
								  	Motor5_Position, Motor5_Current),
						 	
						 	motor5 (Motor6_Direction1, Motor6_Direction2, 
								 	Motor6_PWM, Motor6_Mux, 
								  	Motor6_Position, Motor6_Current),
						 	
						 	motor6 (Motor7_Direction1, Motor7_Direction2, 
								 	Motor7_PWM, Motor7_Mux, 
								  	Motor7_Position, Motor7_Current),
						 	
						 	motor7 (Motor8_Direction1, Motor8_Direction2, 
								 	Motor8_PWM, Motor8_Mux, 
								  	Motor8_Position, Motor8_Current)
{
	// Set up publisher to topic "motor_states"
	mpub = nh.advertise<chuchu_onboard::Int16ArrayHeader>("motor_states", 10);

	// Set up subscriber to topic "commanded_positions"
	speedsub = nh.subscribe("commanded_positions", 100, 
							&motor_node::position_callback, this);

	// Initialize open server
	openSrv = nh.advertiseService("open_hand", &motor_node::open, this);

	// Set up timers
	timer10Hz = nh.createTimer(ros::Duration(0.1), 
							&motor_node::timer10HzCallback, this);
	timer100Hz = nh.createTimer(ros::Duration(0.01), 
							&motor_node::timer100HzCallback, this);

	// Open hand
	motor1.set_setpoint(90);
	motor2.set_setpoint(10);
	motor3.set_setpoint(10);
	motor4.set_setpoint(10);
	motor5.set_setpoint(10);
	motor6.set_setpoint(10);
	motor7.set_setpoint(10);
	motor8.set_setpoint(10);
}

// Function to publish motor states
void motor_node::publish_state()
{	
	// Create array for 8 motors
	chuchu_onboard::Int16ArrayHeader msg;
	// Append positions to array
	// CHANGE TO READ MOTOR POSITIONS
	msg.data.push_back(motor1.currentPosition);
	msg.data.push_back(motor2.currentPosition);
	msg.data.push_back(motor3.currentPosition);
	msg.data.push_back(motor4.currentPosition);
	msg.data.push_back(motor5.currentPosition);
	msg.data.push_back(motor6.currentPosition);
	msg.data.push_back(motor7.currentPosition);
	msg.data.push_back(motor8.currentPosition);

	// Append currents to array
	msg.data.push_back(motor1.currentCurrent);
	msg.data.push_back(motor2.currentCurrent);
	msg.data.push_back(motor3.currentCurrent);
	msg.data.push_back(motor4.currentCurrent);
	msg.data.push_back(motor5.currentCurrent);
	msg.data.push_back(motor6.currentCurrent);
	msg.data.push_back(motor7.currentCurrent);
	msg.data.push_back(motor8.currentCurrent);

	// Add time stamp
	msg.header.stamp = ros::Time::now();

    // Publish message
    mpub.publish(msg);
}

// Set joint speeds
void motor_node::position_callback(const chuchu_onboard::Int16ArrayHeader& msg){
	// Check length of vector 
	if(msg.data.size() == 8){
		// Set motor speeds
		motor1.set_setpoint(msg.data[0]);
		motor2.set_setpoint(msg.data[1]);
		motor3.set_setpoint(msg.data[2]);
		motor4.set_setpoint(msg.data[3]);
		motor5.set_setpoint(msg.data[4]);
		motor6.set_setpoint(msg.data[5]);
		motor7.set_setpoint(msg.data[6]);
		motor8.set_setpoint(msg.data[7]);
	}
}

// Run all motors
void motor_node::run_all(){
	// Run all motors
	motor1.run_position();
	motor2.run_position();
	motor3.run_position();
	motor4.run_position();
	motor5.run_position();
	motor6.run_position();
	motor7.run_position();
	motor8.run_position();
}

// Update PID parameters
void motor_node::update_pid(){
	float p, i, d;
	// Check if parameters are valid (returns TRUE if parameter exists)
	if(ros::param::get("/pid_gains/P", p) &
	   ros::param::get("/pid_gains/I", i) &
	   ros::param::get("/pid_gains/D", d)){
	   	// Set PID parameters for each motor
	   	// PID parameters set in launch file
	   	motor1.set_pid(p,i,d);
	   	motor2.set_pid(p,i,d);
	   	motor3.set_pid(p,i,d);
	   	motor4.set_pid(p,i,d);
	   	motor5.set_pid(p,i,d);
	   	motor6.set_pid(p,i,d);
	   	motor7.set_pid(p,i,d);
	   	motor8.set_pid(p,i,d);
	}
}

// Open hand - called by service (Blocking)
bool motor_node::open(std_srvs::SetBool::Request &req,
					  std_srvs::SetBool::Response &res){
	// Create loop parameters
	ros::Rate looprate = 10;
	int loops = 0;
	bool done = false;
	// Move motors to open position
	motor1.set_setpoint(90);
	motor2.set_setpoint(10);
	motor3.set_setpoint(10);
	motor4.set_setpoint(10);
	motor5.set_setpoint(10);
	motor6.set_setpoint(10);
	motor7.set_setpoint(10);
	motor8.set_setpoint(10);
	// Set goal flags to false
	motor1.atGoal = false;
	motor2.atGoal = false;
	motor3.atGoal = false;
	motor4.atGoal = false;
	motor5.atGoal = false;
	motor6.atGoal = false;
	motor7.atGoal = false;
	motor8.atGoal = false;

	// Blocking loop
	while(1){
		// Run motors
		ROS_INFO("RUNNING: %i", loops);
		run_all();

		// Get done state of all motors
		done = (motor1.atGoal & motor2.atGoal & motor3.atGoal & motor4.atGoal &
				motor5.atGoal & motor6.atGoal & motor7.atGoal & motor8.atGoal);

		// Check if all motors have reached positions
		if(done){
			// If motors reach position, respond with true
			res.success = true;
			res.message = "Hand Opened";
			return true;
		}
		// Set timeout at ~ 10 seconds with 100 Hz loop
		else if(loops > 100){
			// If timeout exceeded, respond with false
			res.success = false;
			res.message = "Timeout on open";
			return true;
		}
		// Delay 10ms 
		looprate.sleep();

		// Increment loops
		loops += 1;
	}	
}

// 10Hz callback
void motor_node::timer10HzCallback(const ros::TimerEvent& event){
	// Publish positions and currents
	publish_state();

	// Update PID parameters from server
	//update_pid();
}

// 100Hz callback
void motor_node::timer100HzCallback(const ros::TimerEvent& event){
	run_all();
}

