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
#include <tactile_node/MPL115A2.h>

MPL115A2::MPL115A2(i2c* _ptr) : ptr(_ptr),
								calibrationFlag(false),
								calibrationPressure(0),
								lastPressure(0)
{
	// Get temperature coefficients 
	// (Implemented in tactile_sensor constructor)
	//get_temp_coefficients();

	// Initialize lastPressure (must call when i2c is open, use lastPres_init)
	// lastPressure = get_raw_pressure();

	// Calibrate
	// calibrate();
}

void MPL115A2::lastPres_init(){
	lastPressure = get_raw_pressure();
}

float MPL115A2::update_pressure(){
	// Temporary variables for pressure and temperature
	float currentPressure = get_raw_pressure();
	float temperature = get_raw_temperature();

	// Unwrap pressure data
	float delta_1 = lastPressure - currentPressure;
	float delta_2 = lastPressure + (1023.0F - currentPressure);

	// Take smaller difference
	if(std::abs(delta_1) < std::abs(delta_2)){
		currentPressure = lastPressure - delta_1;
	}
	else{
		currentPressure = lastPressure - delta_2;
	}

	// Update last pressure
	lastPressure = currentPressure;

	// Temperature compensation
	currentPressure = (A0 + (B1 + C12*(temperature))*(currentPressure) + 
					   B2*(temperature));

	// Convert to kPa (formula from datasheet) used for debug
	//currentPressure = currentPressure * (65.0F/1023.0F) + 50.0;

	// If sensor has been calibrated, apply offset
	if(calibrationFlag){
		currentPressure = currentPressure - calibrationPressure;
	}

	// Return the current pressure value
	return currentPressure;
}

float MPL115A2::get_raw_pressure(){
	//write anything to CONVERT to begin 
	ptr -> write_byte_reg(SENSORADDR, CONVERT, 0x00);
	
	//sleep for 5 ms (3 ms minimum)
	usleep(5000);

	// Read pressure
	int16_t pres =	((ptr -> read_reg_byte(SENSORADDR, PRES_MSB) << 8) | 
				  	  ptr -> read_reg_byte(SENSORADDR, PRES_LSB)) >> 6;
	
	// Return cast as float
	return pres;
}

float MPL115A2::get_raw_temperature(){
	// Read temperature
	int16_t temp = 	((ptr -> read_reg_byte(SENSORADDR, TEMP_MSB) << 8) |
				  	  ptr -> read_reg_byte(SENSORADDR, TEMP_LSB)) >> 6;
	
	// Return cast as float
	return temp;
}

void MPL115A2::get_temp_coefficients(){
	// Create variables for raw data
	int16_t A0_bits, B1_bits, B2_bits, C12_bits;

	// Read coefficient values from sensor
	A0_bits =	(ptr -> read_reg_byte(SENSORADDR, A0_MSB) << 8) | 
				 ptr -> read_reg_byte(SENSORADDR, A0_LSB);
	B1_bits =	(ptr -> read_reg_byte(SENSORADDR, B1_MSB) << 8) | 
			   	 ptr -> read_reg_byte(SENSORADDR, B1_LSB);
	B2_bits = 	(ptr -> read_reg_byte(SENSORADDR, B2_MSB) << 8) | 
			   	 ptr -> read_reg_byte(SENSORADDR, B2_LSB);
	C12_bits = 	(ptr -> read_reg_byte(SENSORADDR, C12_MSB) << 8) | 
				 ptr -> read_reg_byte(SENSORADDR, C12_LSB);

	//Convert A0 to float (1 sign bit, 12 integer bits, 3 fractional) A0 = A0_bits / 2^3
	A0 = (float)A0_bits/8.0;

	//Convert B1 to float (1 sign bit, 2 integer bits, 13 fractional) B1 = B1_bits / 2^13
	B1 = (float)B1_bits/(8192.0);

	//Convert B2 to float (1 sign bit, 1 integer bits, 14 fractional) B2 = B2_bits / 2^14
	B2 = (float)B2_bits/(16384.0);

	//Convert A0 to float (1 sign bit, 0 integer bits, 13 fractional, 2 zeros in LSB, 9 zero pad) C12 = C12_bits >> 2 / 2^22
	C12 = ((float)(C12_bits >> 2)/(4194304.0));
}

void MPL115A2::calibrate(){
	// Set calibration flag to false
	calibrationFlag = false;

	// Update calibration attribute
	calibrationPressure = update_pressure();

	// Set calibration flag
	calibrationFlag = true;
}