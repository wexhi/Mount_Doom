#ifndef __PID_H	
#define __PID_H

#include <math.h>
typedef struct
{
	float target_val;
	float actual_val;
	float err;
	float err_last;
	float err_sum;
	float Kp, Ki, Kd;
	float err_prev;
}tPid;

typedef struct
{
	float target_pos;
	float actual_pos;
	float err;
	float err_last;
	float err_sum;
	float Kp, Ki, Kd;
	
}Car;

void PID_init(void);

float PID_realize(tPid * pid, float actual_val);
float PID_Anglerealize(tPid * pid, float actual_val);
int Position_PID(Car * pid, float target_val, float actual_val);
int Increnental_PID(tPid * pid, float target_val, float actual_val);

#endif // !__PID_H

