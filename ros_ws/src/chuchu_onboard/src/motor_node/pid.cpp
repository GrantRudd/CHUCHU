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
#include <motor_node/pid.h>

PID::PID(float Kp, float Ki, float Kd): Kp(Kp),Ki(Ki),Kd(Kd)
{
	setPoint = 0;
	lastError = 0;
	integral = 0;
	iLimit = 10*10^9;
}

float PID::Update_Value(float current) 
{
	float val;
	//calculate error
	float error = setPoint - current;
	//update integral
	integral += error;
	//check for integral windup
	if (abs(integral) > iLimit) 
	{
		integral = iLimit * (integral / abs(integral));
	}
	//update PID
	val = Kp*error + Ki*integral + Kd*(error - lastError);
	//update last error
	lastError = error;

	return val;
}

void PID::set_coeffs(float p, float i, float d){
	// Set coefficients to new values
	Kp = p;
	Ki = i;
	Kd = d;
}

void PID::print_coeffs(){
	cout << "P: " << Kp << " I: " << Ki << " D: " << Kd << endl;
}