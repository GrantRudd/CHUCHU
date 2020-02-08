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
#include <tactile_node/i2c.h>

i2c::i2c(){
	// Open I2C on I2C-1
	file = open("/dev/i2c-1", O_RDWR);
}

void i2c::write_byte(uint8_t addr, uint8_t data){
	// Set address to sensor address
	ioctl(file, I2C_SLAVE, addr);

	// Write one byte of data
	i2c_smbus_write_byte(file, data);
}

void i2c::write_byte_reg(uint8_t addr, uint8_t reg, uint8_t data){
	// Set address to sensor address
	ioctl(file, I2C_SLAVE, addr);

	// Write one byte of data to a register
	i2c_smbus_write_byte_data(file, reg, data);
}

int i2c::read_reg_byte(int addr, int reg){
	//Set slave address
	ioctl(file, I2C_SLAVE, addr);

	//read byte
	int data = i2c_smbus_read_byte_data(file, reg); 

	//return data
	return data;
}