/*
 * PID.c
 *
 *  Created on: Oct 3, 2023
 *      Author: User
 */
#include"PID.h"
#include"gpio.h"
#include"tim.h"
void PIDController_Init(PIDController*pid){
	//Clear controller variables
	pid->integrator=0.0f;
	pid->prevError=0.0f;

	pid->differentiator=0.0f;
	pid->prevMeasurement=0.0f;
	pid->out=0.0f;
}

float  PIDController_Update(PIDController*pid, float setpoint, float measurement, float Kp, float Ki, float Kd){
	//Error signal
	float error=setpoint-measurement;
	//Proportional
	float proportional=Kp*error;
	//Integral
	pid->integrator=pid->integrator+0.5f*Ki*pid->T*(error+pid->prevError);
	//Anti-wind-up via integrator clamping
	if(pid->integrator>pid->limMaxInt){
		pid->integrator=pid->limMaxInt;
	}
	else if(pid->integrator<pid->limMinInt){
		pid->integrator=pid->limMinInt;
	}

	//Derivative (band-limited differntiator)
	pid->differentiator=-(2.0f*Kd*(measurement-pid->prevMeasurement)+(2.0f*pid->tau-pid->T)*pid->differentiator)/(2.0f*pid->tau+pid->T);
	//Compute output and apply limits
	pid->out = proportional + pid->integrator + pid->differentiator;
	if (pid->out > pid->limMax) {
	 pid->out = pid->limMax;
	 } else if (pid->out < pid->limMin) {
	 pid->out = pid->limMin;
	 }
	//Store error and measurement for later use
	 pid->prevError = error;
	 pid->prevMeasurement = measurement;
	 //Return controller output
	 return pid->out;

}

