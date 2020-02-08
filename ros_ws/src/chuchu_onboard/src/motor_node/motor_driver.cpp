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

#include <motor_node/motor_driver.h>

Motor::Motor(std::string Dir1, std::string Dir2, std::string PWMPin,
			 std::string MuxEnablePin, int PosMuxSelect, int CurrentMuxSelect):
			 Dir1(Dir1), Dir2(Dir2), PWMPin(PWMPin), MuxEnablePin(MuxEnablePin),
			 PosMuxSelect(PosMuxSelect), CurrentMuxSelect(CurrentMuxSelect), 
			 pid(1.6, 0.0, 0.5), atGoal(true) {
	// Initialize appropriate GPIO pins
	ChuPIO::gpio_init(Mux_A, ChuPIO::OUTPUT);
	ChuPIO::gpio_init(Mux_B, ChuPIO::OUTPUT);
	ChuPIO::gpio_init(Mux_C, ChuPIO::OUTPUT);
	ChuPIO::gpio_init(MuxEnablePin, ChuPIO::OUTPUT);
	// Disable all mux channels upon initialization
	ChuPIO::digital_write(MuxEnablePin, ChuPIO::HIGH);
	ChuPIO::gpio_init(Dir1, ChuPIO::OUTPUT);
	ChuPIO::gpio_init(Dir2, ChuPIO::OUTPUT);
	ChuPIO::pwm_init(PWMPin);
	ChuPIO::run_pwm(PWMPin);

	// Turn motor off
	ChuPIO::digital_write(Dir1, ChuPIO::LOW);
	ChuPIO::digital_write(Dir2, ChuPIO::LOW);
	ChuPIO::set_duty_cycle(PWMPin, 0);

	// Get initial value for current/last position and force
	currentPosition = get_position();
	currentCurrent = get_current();
	lastPosition = currentPosition;
}

void Motor::stop(void) {
	// Turn motor off
	ChuPIO::digital_write(Dir1, ChuPIO::LOW);
	ChuPIO::digital_write(Dir2, ChuPIO::LOW);
	ChuPIO::set_duty_cycle(PWMPin, 0);
}

void Motor::run_position() {
	//Read current position
	currentPosition = get_position();

	//read current force
	currentCurrent = get_current();

	//clear goal flag
	atGoal = false;

	//update PID controller with current position
	int x = int(pid.Update_Value(currentPosition));

	//DEBUG
	//cout << "Position: " << currentPosition << endl;
	//cout << "PWM VALUE: " << x << endl;
	//pid.print_coeffs();
	//cout << "*------------------------------------*" << endl;
	//Set motor speed and directions
	if (x > STOP_THRESHOLD) {
		//Set direction of motor
		ChuPIO::digital_write(Dir1, ChuPIO::HIGH);
		ChuPIO::digital_write(Dir2, ChuPIO::LOW);
		//Check velocity, modify if outside limits
		if (x > 100)
			x = 100;
		else if (x < MINIMUM_PWM)
			x = MINIMUM_PWM;
		//Set PWM value
		ChuPIO::set_duty_cycle(PWMPin, x);
	} else if (x < -(STOP_THRESHOLD)) {
		//Set direction of motor
		ChuPIO::digital_write(Dir1, ChuPIO::LOW);
		ChuPIO::digital_write(Dir2, ChuPIO::HIGH);
		//Take absolute value of velocity
		x = abs(x);
		//Check velocity, modify if outside limits
		if (x > 100)
			x = 100;
		else if (x < MINIMUM_PWM)
			x = MINIMUM_PWM;
		//Set PWM value
		ChuPIO::set_duty_cycle(PWMPin, x);

	} else {
		stop();

		//set goal flag
		atGoal = true;
	}
}

float Motor::get_position(void) {
	// Bitmask mux select value to get appropriate states for position mux pins
	ChuPIO::VALUE pin0 = ChuPIO::VALUE(PosMuxSelect & 0b001);
	ChuPIO::VALUE pin1 = ChuPIO::VALUE((PosMuxSelect & 0b010) >> 1);
	ChuPIO::VALUE pin2 = ChuPIO::VALUE((PosMuxSelect & 0b100) >> 2);

	// Set mux pins to appropriate state
	ChuPIO::digital_write(Mux_A, pin0);
	ChuPIO::digital_write(Mux_B, pin1);
	ChuPIO::digital_write(Mux_C, pin2);

	// Enable mux
	ChuPIO::digital_write(MuxEnablePin, ChuPIO::LOW);

	// Calculate position as a value between 0-100
	float pos = (float(ChuPIO::analog_read(APin)) * 100.0 / 4096.0);

	// Disable mux
	ChuPIO::digital_write(MuxEnablePin, ChuPIO::HIGH);

	return pos;
}

float Motor::get_current(void) {
	// Bitmask mux select value to get appropriate states for position mux pins
	ChuPIO::VALUE pin0 = ChuPIO::VALUE(CurrentMuxSelect & 0b001);
	ChuPIO::VALUE pin1 = ChuPIO::VALUE((CurrentMuxSelect & 0b010) >> 1);
	ChuPIO::VALUE pin2 = ChuPIO::VALUE((CurrentMuxSelect & 0b100) >> 2);

	// Set mux pins to appropriate state
	ChuPIO::digital_write(Mux_A, pin0);
	ChuPIO::digital_write(Mux_B, pin1);
	ChuPIO::digital_write(Mux_C, pin2);

	// Enable mux
	ChuPIO::digital_write(MuxEnablePin, ChuPIO::LOW);

	// Read voltage
	float voltage = float(ChuPIO::analog_read(APin)) * 1.8 / 4096.0;
	// Convert voltage to current (mA)
	float current = abs((0.8995 - voltage) / 0.0041);

	//disable mux
	ChuPIO::digital_write(MuxEnablePin, ChuPIO::HIGH);

	return current;
}

void Motor::set_setpoint(float setPoint) {
	//range check setpoint value
	if ((setPoint >= 0) && (setPoint <= 100))
		pid.setPoint = setPoint;
}

float Motor::get_setpoint(){
	return pid.setPoint;
}

// Update PID values
void Motor::set_pid(float p, float i, float d){
	pid.set_coeffs(p, i, d);
}