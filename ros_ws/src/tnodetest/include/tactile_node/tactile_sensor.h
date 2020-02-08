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
#ifndef tactile_sensor_H_
#define tactile_sensor_H_

#include <vector>
#include <tactile_node/MPL115A2.h>

#define MUXBASE		0x20
#define MPL_1		0x01
#define MPL_2		0x02
#define MPL_3		0x04
#define MPL_4		0x08
#define ALL_OFF		0x00

class tactile_sensor{
public:
	tactile_sensor(i2c* _ptr, int _offset);
	void update_pressures();
	void calibrate();

	std::vector<float> sensorVals;

private:
	// pointer to i2c file
	i2c* ptr;

	// Instances of MPL115A2 Sensors
	MPL115A2 mpl_1;
	MPL115A2 mpl_2;
	MPL115A2 mpl_3;
	MPL115A2 mpl_4;

	// Offset
	int offset;
};

#endif