/* 
 * File:   PID.h
 * Author: Zac
 *
 * Created on May 12, 2017, 6:53 AM
 */

#ifndef PID_H
#define	PID_H

typedef struct 
{
    float _kp, _ki, _kd;
    float _dt;
    float _output;
    float _error,_prevError,_dError;
    int _now, _past;
    float _derivative, _integral;
    float _target;
    int* _targRef;
}PID_Struct_t;


void INIT_PID(PID_Struct_t* _this,float target, float kp, float ki, float kd);

/******************************************************************
This is where you will declare all of the methods
that will be able to be called by other segments of code externally
******************************************************************/

int updateOutput(PID_Struct_t *_this, float sample);
void setPropotionality(PID_Struct_t *_this, float kp, float ki, float kd);
void clearIntegral(PID_Struct_t *_this);
void updateTarget(PID_Struct_t *_this, float target);
float readError(PID_Struct_t *_this);
float readDerivative(PID_Struct_t *_this);
float readIntegral(PID_Struct_t *_this);
float readOutput(PID_Struct_t *_this);

void clearSystem(PID_Struct_t *_this);
float returnTarget(PID_Struct_t *_this);




#endif	/* PID_H */

