#include "pid.h"


tPid pidMotor1Speed, pidMotor2Speed, pidMPU6050YawMovement, pidMPU6050PitchMovement;
Car  wheel1, wheel2;

void PID_init(void)
{
	pidMotor1Speed.target_val = 0;
	pidMotor1Speed.actual_val = 0;
	pidMotor1Speed.err = 0;
	pidMotor1Speed.err_last = 0;
	pidMotor1Speed.err_sum = 0;
	pidMotor1Speed.err_prev = 0;
	pidMotor1Speed.Kp = 1200;
	pidMotor1Speed.Ki = 25;
	pidMotor1Speed.Kd = 0;
	
	pidMotor2Speed.target_val = 0;
	pidMotor2Speed.actual_val = 0;
	pidMotor2Speed.err = 0;
	pidMotor2Speed.err_last = 0;
	pidMotor2Speed.err_sum = 0;
	pidMotor2Speed.err_prev = 0;
	pidMotor2Speed.Kp = 2550;
	pidMotor2Speed.Ki = 35;
	pidMotor2Speed.Kd = 0;
	
	wheel1.target_pos = 0;
	wheel1.actual_pos = 0;
	wheel1.err = 0;
	wheel1.err_last = 0;
	wheel1.err_sum = 0;
	wheel1.Kp = 0.5;
	wheel1.Ki = 0.3;
	wheel1.Kd = 0;
	
	wheel2.target_pos = 0;
	wheel2.actual_pos = 0;
	wheel2.err = 0;
	wheel2.err_last = 0;
	wheel2.err_sum = 0;
	wheel2.Kp = 0.5;
	wheel2.Ki = 0.3;
	wheel2.Kd = 0;
	
	pidMPU6050YawMovement.target_val = 0;
	pidMPU6050YawMovement.actual_val = 0;
	pidMPU6050YawMovement.err = 0;
	pidMPU6050YawMovement.err_last = 0;
	pidMPU6050YawMovement.err_sum = 0;
	pidMPU6050YawMovement.Kp = 0.02;
	pidMPU6050YawMovement.Ki = 0;
	pidMPU6050YawMovement.Kd = 0.2;
	
	pidMPU6050PitchMovement.target_val = 0;
	pidMPU6050PitchMovement.actual_val = 0;
	pidMPU6050PitchMovement.err = 0;
	pidMPU6050PitchMovement.err_last = 0;
	pidMPU6050PitchMovement.err_sum = 0;
	pidMPU6050PitchMovement.Kp = 0.08;
	pidMPU6050PitchMovement.Ki = 0;
	pidMPU6050PitchMovement.Kd = 0.1;
}

float PID_realize(tPid * pid, float actual_val)
{
	if (pid->target_val == 0)
	{
		return 0;
	}
	
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	
	
	pid->err_sum += pid->err;
	if (pid->err_sum > 1000) pid->err_sum = 1000;
	if (pid->err_sum < -1000) pid->err_sum = -1000;
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
	
	//pid->actual_val += pid->Kp * (pid->err - pid->err_last) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->err_last + pid->err_prev);
	pid->err_prev = pid->err_last;
	pid->err_last = pid->err;

	return pid->actual_val;
}

float PID_Anglerealize(tPid * pid, float actual_val)
{
	

	
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	

	pid->err_sum += pid->err;
	if (pid->err_sum > 1000) pid->err_sum = 1000;
	if (pid->err_sum < -1000) pid->err_sum = -1000;
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
	
	//pid->actual_val += pid->Kp * (pid->err - pid->err_last) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->err_last + pid->err_prev);
	pid->err_prev = pid->err_last;
	pid->err_last = pid->err;
	
	if (fabs(pid->actual_val) <= 0.05)
	{
		return 0;
	}
	

	return pid->actual_val;
}

long Num_Encoder_Cnt(float num)
{
	return num * 1560;
}

int Position_PID(Car * pid, float target_val, float actual_val)
{
	pid->target_pos = Num_Encoder_Cnt(target_val);
	pid->actual_pos = actual_val;
	pid->err = pid->target_pos - pid->actual_pos;
	if (fabs(pid->err) < 100)
	{
		pid->err_sum = 0;
		pid->err_last = 0;
		return 0;
	}
	else
	{
		pid->err_sum += pid->err;
		if (pid->err_sum > 5000) pid->err_sum = 5000;
		if (pid->err_sum < -5000) pid->err_sum = -5000;
		pid->actual_pos = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
		pid->err_last = pid->err;
	
		return pid->actual_pos;
	}
}
