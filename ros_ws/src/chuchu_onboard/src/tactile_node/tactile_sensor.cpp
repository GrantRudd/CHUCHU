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
#include<tactile_node/tactile_sensor.h>

tactile_sensor::tactile_sensor(i2c* _ptr, int _offset) : mpl_1(_ptr),
														 mpl_2(_ptr),
														 mpl_3(_ptr),
														 mpl_4(_ptr)	
{
	// Initialize Pointer
	ptr = _ptr;

	// Initialize offset
	offset = _offset;

	// Get temperature coefficients and calibrate all sensors
	// Sensor 1
	ptr -> write_byte(MUXBASE + offset, MPL_1);
	mpl_1.get_temp_coefficients();
	mpl_1.lastPres_init();
	//mpl_1.calibrate();
	
	// Sensor 2
	ptr -> write_byte(MUXBASE + offset, MPL_2);
	mpl_2.get_temp_coefficients();
	mpl_2.lastPres_init();
	//mpl_2.calibrate();

	// Sensor 3
	ptr -> write_byte(MUXBASE + offset, MPL_3);
	mpl_3.get_temp_coefficients();
	mpl_3.lastPres_init();
	//mpl_3.calibrate();

	// Sensor 4
	ptr -> write_byte(MUXBASE + offset, MPL_4);
	mpl_4.get_temp_coefficients();
	mpl_4.lastPres_init();
	//mpl_4.calibrate();
	
	// Disable all I2C devices
	ptr -> write_byte(MUXBASE + offset, ALL_OFF);


}

void tactile_sensor::update_pressures(){
	// Temporary variable to store pressure
	float p;

	// Clear value vector
	sensorVals.clear();

	// Enable MPL115A2 on channel 1
	ptr -> write_byte(MUXBASE + offset, MPL_1);

	// Get pressure and store in value vector
	p = mpl_1.update_pressure();
	sensorVals.push_back(p);
	
	// Enable MPL115A2 on channel 2
	ptr -> write_byte(MUXBASE + offset, MPL_2);

	// Get pressure and store in value vector
	p = mpl_2.update_pressure();
	sensorVals.push_back(p);

	// Enable MPL115A2 on channel 3
	ptr -> write_byte(MUXBASE + offset, MPL_3);

	// Get pressure and store in value vector
	p = mpl_3.update_pressure();
	sensorVals.push_back(p);

	// Enable MPL115A2 on channel 4
	ptr -> write_byte(MUXBASE + offset, MPL_4);

	// Get pressure and store in value vector
	p = mpl_4.update_pressure();
	sensorVals.push_back(p);
	
	// Disable all I2C devices
	ptr -> write_byte(MUXBASE + offset, ALL_OFF);
}

void tactile_sensor::calibrate(){
	// Open I2C and calibrate 1st sensor
	ptr -> write_byte(MUXBASE + offset, MPL_1);
	mpl_1.calibrate();

	// Open I2C and calibrate 2nd sensor
	ptr -> write_byte(MUXBASE + offset, MPL_2);
	mpl_2.calibrate();

	// Open I2C and calibrate 3rd sensor
	ptr -> write_byte(MUXBASE + offset, MPL_3);
	mpl_3.calibrate();

	// Open I2C and calibrate 4th sensor
	ptr -> write_byte(MUXBASE + offset, MPL_4);
	mpl_4.calibrate();

	// Disable all I2C devices
	ptr -> write_byte(MUXBASE + offset, ALL_OFF);
}