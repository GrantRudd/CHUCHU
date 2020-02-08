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
 *
 *
 *ChuPIO Library: Simple GPIO library to facilitate GPIO, PWM and ADC
 *functionaity for Beaglebone Black. (TESTED: Ubuntu 14.04.3 kernel 4.1.x)
 *
 *Created by: Grant Rudd
 *Rev Date: 11/22/2017
 *
 */

#include <motor_node/ChuPIO.h>

//GPIO export numbers
map<string, int> GPIO_EXPORT;
//PWM export numbers
map<string, int> PWM_EXPORT;
//ADC name map
map<string, string> ADC_PINS;

namespace ChuPIO {

/* GPIO FUNCTIONS */

//Initialize a pin as a GPIO pin and set direction
int gpio_init(string pinName, DIRECTION direction) {
	stringstream s;
	string path;

	//set pinmux to GPIO (echo to /sys/devices/platform/ocp.3/PX_XX_pinmux.XX/state)
	int i = 0;
	int ret = 1;
	//kludge to fix changing pinmux filenames... find more elegant solution
	while (ret == 1) {
		s << PIN_PATH << pinName << "_pinmux." << i << "/state";
		path = s.str();
		ret = write(path, "gpio");
		s.str("");
		if (i > 100) {
			cout << "Path Broken: " << pinName << endl;
			return 1;
		}
		i++;
	}
	//set direction (echo to /sys/class/gpio/gpioXX/direction)
	s.str("");
	s << GPIO_PATH << "gpio" << GPIO_EXPORT[pinName] << "/direction";
	path = s.str();

	switch (direction) {
	case INPUT:
		write(path, "in");
		break;
	case OUTPUT:
		write(path, "out");
		break;
	default:
		break;
	}
	return 0;
}
//Set the value (HIGH or LOW) of a GPIO pin
int digital_write(string pinName, VALUE value) {
	stringstream s;
	string path;

	//set value (echo to /sys/class/gpio/gpioXX/value)
	s << GPIO_PATH << "gpio" << GPIO_EXPORT[pinName] << "/value";
	path = s.str();

	switch (value) {
	case LOW:
		write(path, "0");
		break;
	case HIGH:
		write(path, "1");
		break;
	default:
		break;
	}

	return 0;
}
//Read value of digital input pin (returns true or false)
bool digital_read(string pinName) {
	stringstream s;
	string path;

	//read value (cat to /sys/class/gpio/gpioXX/value)
	s << GPIO_PATH << "gpio" << GPIO_EXPORT[pinName] << "/value";
	path = s.str();

	string inp = read(path);

	if (inp == "0") {
		return false;
	} else {
		return true;
	}
}

/* PWM FUNCTIONS */
//setup a pin as PWM (MUST BE BBB PWM PIN, SEE DOCUMENTATION)
//frequency is initialized to 1000Hz.
int pwm_init(string pinName) {
	stringstream s;
	string path;
	int exp = PWM_EXPORT[pinName];

	//set pinmux to PWM (echo to /sys/devices/ocp.3/PX_XX_pinmux.XX/state)
	//note: if using P9_42, echo pwm2
	int i = 0;
	int ret = 1;
	//kludge to fix changing pinmux filenames... find more elegant solution
	while (ret == 1) {
		s << PIN_PATH << pinName << "_pinmux." << i << "/state";
		path = s.str();
		if (exp == 7) {
			ret = write(path, "pwm2");
		} else {
			ret = write(path, "pwm");
		}
		s.str("");

		if (i > 100) {
			cout << "Path Broken: " << pinName << endl;
			return 1;
		}
		i++;
	}

	//export PWM pin
	s.str("");
	s << PWM_PATH << "export";
	path = s.str();

	write_int(path, exp);

	//set frequency to 1000Hz
	s.str("");
	s << PWM_PATH << "pwm" << exp << "/" << PWM_PERIOD;
	path = s.str();

	write_int(path, F1000HZ);

	return 0;
}
//Set the duty cycle of a pin, in terms of percentage (0-100)
int set_duty_cycle(string pinName, int dutyCycle) {
	stringstream s;
	string path;
	s << PWM_PATH << "pwm" << PWM_EXPORT[pinName] << "/" << PWM_DUTY;
	path = s.str();

	//calculate duty time on (in nS)
	int dc = int((float(dutyCycle) / 100.0) * float(F1000HZ));

	write_int(path, dc);

	return 0;
}

//enable pwm
int run_pwm(string pinName) {
	stringstream s;
	string path;
	s << PWM_PATH << "pwm" << PWM_EXPORT[pinName] << "/" << PWM_RUN;
	path = s.str();
	write(path, "1");
	return 0;
}

/* ADC Functions */
//Read ADC and return 12-bit ADC value
int stop_pwm(string pinName) {
	stringstream s;
	string path;
	s << PWM_PATH << "pwm" << PWM_EXPORT[pinName] << "/" << PWM_RUN;
	path = s.str();
	write(path, "0");
	return 0;
}

//ADC functions
int analog_read(string pinName) {
	stringstream s;
	string path;
	s << ADC_PATH << ADC_PINS[pinName];
	path = s.str();

	string val = read(path);

	return atoi(val.c_str());
}
//File I/O functions
int write(string filePath, string value) {
	ofstream fs;
	fs.open(filePath.c_str());
	if (!fs.is_open()) {
		//cout << "Filepath:" << filePath << " not found" << endl;
		return 1;
	}
	fs << value;
	fs.close();
	return 0;
}

int write_int(string filePath, int value) {
	stringstream s;
	s << value;
	write(filePath, s.str());
	return 0;
}

string read(string filePath) {
	string read;
	ifstream fs;
	fs.open(filePath.c_str());
	if (!fs.is_open()) {
		//cout << "Filepath:" << filePath << " not found" << endl;
		return "1";
	}

	getline(fs, read);
	fs.close();
	return read;
}

//Initialize all necessary pin maps
void ChuPIO_init(void) {
	//Initialize GPIO_EXPORT map
	GPIO_EXPORT["P8_07"] = 66;
	GPIO_EXPORT["P8_08"] = 67;
	GPIO_EXPORT["P8_09"] = 69;
	GPIO_EXPORT["P8_10"] = 68;
	GPIO_EXPORT["P8_11"] = 45;
	GPIO_EXPORT["P8_12"] = 44;
	GPIO_EXPORT["P8_13"] = 23;
	GPIO_EXPORT["P8_14"] = 26;
	GPIO_EXPORT["P8_15"] = 47;
	GPIO_EXPORT["P8_16"] = 46;
	GPIO_EXPORT["P8_17"] = 27;
	GPIO_EXPORT["P8_18"] = 65;
	GPIO_EXPORT["P8_19"] = 22;
	GPIO_EXPORT["P8_26"] = 61;
	GPIO_EXPORT["P8_27"] = 86;
	GPIO_EXPORT["P8_28"] = 88;
	GPIO_EXPORT["P8_29"] = 87;
	GPIO_EXPORT["P8_30"] = 89;
	GPIO_EXPORT["P8_31"] = 10;
	GPIO_EXPORT["P8_32"] = 11;
	GPIO_EXPORT["P8_33"] = 9;
	GPIO_EXPORT["P8_34"] = 81;
	GPIO_EXPORT["P8_35"] = 8;
	GPIO_EXPORT["P8_36"] = 80;
	GPIO_EXPORT["P8_37"] = 78;
	GPIO_EXPORT["P8_38"] = 79;
	GPIO_EXPORT["P8_39"] = 76;
	GPIO_EXPORT["P8_40"] = 77;
	GPIO_EXPORT["P8_41"] = 74;
	GPIO_EXPORT["P8_42"] = 75;
	GPIO_EXPORT["P8_43"] = 72;
	GPIO_EXPORT["P8_44"] = 73;
	GPIO_EXPORT["P8_45"] = 70;
	GPIO_EXPORT["P8_46"] = 71;

	GPIO_EXPORT["P9_11"] = 30;
	GPIO_EXPORT["P9_12"] = 60;
	GPIO_EXPORT["P9_13"] = 31;
	GPIO_EXPORT["P9_14"] = 50;
	GPIO_EXPORT["P9_15"] = 48;
	GPIO_EXPORT["P9_16"] = 51;
	GPIO_EXPORT["P9_17"] = 5;
	GPIO_EXPORT["P9_18"] = 4;
	GPIO_EXPORT["P9_21"] = 3;
	GPIO_EXPORT["P9_22"] = 2;
	GPIO_EXPORT["P9_23"] = 49;
	GPIO_EXPORT["P9_24"] = 15;
	GPIO_EXPORT["P9_25"] = 117;
	GPIO_EXPORT["P9_26"] = 14;
	GPIO_EXPORT["P9_27"] = 115;
	GPIO_EXPORT["P9_28"] = 113;
	GPIO_EXPORT["P9_29"] = 111;
	GPIO_EXPORT["P9_30"] = 112;
	GPIO_EXPORT["P9_31"] = 110;
	GPIO_EXPORT["P9_41"] = 20;
	GPIO_EXPORT["P9_42"] = 7;

	//Initialize PWM_EXPORT map
	PWM_EXPORT["P9_22"] = 0;
	PWM_EXPORT["P9_31"] = 0;

	PWM_EXPORT["P9_21"] = 1;
	PWM_EXPORT["P9_29"] = 1;

	PWM_EXPORT["P9_42"] = 2;

	PWM_EXPORT["P9_14"] = 3;
	PWM_EXPORT["P8_36"] = 3;

	PWM_EXPORT["P9_16"] = 4;
	PWM_EXPORT["P8_34"] = 4;

	PWM_EXPORT["P8_19"] = 5;
	PWM_EXPORT["P8_45"] = 5;

	PWM_EXPORT["P8_13"] = 6;
	PWM_EXPORT["P8_46"] = 6;

	PWM_EXPORT["P9_28"] = 7;

	//Initialize ADC_PINS map
	ADC_PINS["P9_33"] = "in_voltage4_raw";
	ADC_PINS["P9_35"] = "in_voltage6_raw";
	ADC_PINS["P9_36"] = "in_voltage5_raw";
	ADC_PINS["P9_37"] = "in_voltage2_raw";
	ADC_PINS["P9_38"] = "in_voltage3_raw";
	ADC_PINS["P9_39"] = "in_voltage0_raw";
	ADC_PINS["P9_40"] = "in_voltage1_raw";
	}
}