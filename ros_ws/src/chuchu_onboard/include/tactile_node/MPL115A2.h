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
// Revison date: 3/12/18
#pragma once
#ifndef MPL115A2_H_
#define MPL115A2_H_

#include <tactile_node/i2c.h>
#include <unistd.h>
#include <stdint.h>
#include <cmath>
#include <cstdio>

// MPL115A2 Register map
#define SENSORADDR		0x60
#define	CTRLREG1		0x26
#define PT_DATA_CFG		0x13		
#define PRES_MSB		0x00
#define PRES_LSB		0x01
#define TEMP_MSB		0x02
#define TEMP_LSB		0x03
#define A0_MSB			0x04			
#define A0_LSB			0x05
#define B1_MSB			0x06
#define B1_LSB			0x07
#define B2_MSB			0x08
#define B2_LSB			0x09
#define C12_MSB			0x0A
#define C12_LSB			0x0B
#define CONVERT			0x12

class MPL115A2{
private:
	// Read registers
	float get_raw_pressure();
	float get_raw_temperature();

	// Private attributes
	i2c* ptr;
	float lastPressure, calibrationPressure;
	float A0, B1, B2, C12;
	bool calibrationFlag;

public:
	// Constructor
	MPL115A2(i2c* _ptr);

	// Return compensated pressure
	float update_pressure();

	// Update temperature coefficients
	void get_temp_coefficients();

	// (Re)calibrate sensor
	void calibrate();

	// Initialize lastPressure attribute
	void lastPres_init();
};


#endif