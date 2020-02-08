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
#pragma once
#ifndef CHUPIO_H_
#define CHUPIO_H_

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>

using namespace std;

//General Defines
#define PIN_PATH 	"/sys/devices/ocp.3/"
#define DEFAULt 	"default"
#define STATE 		"state"

//GPIO Specific Parameters
#define GPIO_PATH	"/sys/class/gpio/"

//PWM Specific Parameters
#define PWM_PATH	"/sys/class/pwm/"
#define F1000HZ 	1000000 //1000 Hz (period in ns)
#define PWM_PERIOD 	"period_ns"
#define PWM_DUTY	"duty_ns"
#define PWM_RUN		"run"

//ADC Specific Parameters
#define ADC_PATH	"/sys/bus/iio/devices/iio:device0/"

namespace ChuPIO {
//enumerations for GPIO datatypes
enum DIRECTION {
	INPUT, OUTPUT
};
enum VALUE {
	LOW, HIGH
};

//Map Initializer (MUST BE CALLED BEFORE ChuPIO WILL WORK)
void ChuPIO_init(void);

//GPIO function prototypes
int gpio_init(string pinName, DIRECTION direction);
int digital_write(string pinName, VALUE value);
bool digital_read(string pinName);

//PWM function prototypes
int pwm_init(string pinName);
int set_duty_cycle(string pinName, int dutyCycle);
int run_pwm(string pinName);
int stop_pwm(string pinName);

//ADC function prototypes
int analog_read(string pinName);

//File I/O function prototypes
int write(string filePath, string value);
int write_int(string filePath, int value);
string read(string filePath);

}

#endif /* CHUPIO_H_ */
