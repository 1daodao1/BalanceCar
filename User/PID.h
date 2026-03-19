#ifndef __PID_H
#define __PID_H

typedef struct {
	float Target;
	float Actual;
	float Actual1;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	//积分限幅
	float ErrorIntMax;
	float ErrorIntMin;
	
	float OutMax;
	float OutMin;
	
	float OutOffset;//输出偏移值
} PID_t;

void PID_Update(PID_t *p);
void PID_Init(PID_t *p);

#endif
