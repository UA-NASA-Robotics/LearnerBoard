#include "PID.h"
#include "interruptHandler.h"

/******************************************************************
Declare all internal variables and methods that will only be called 
and used internally
******************************************************************/



void INIT_PID(PID_Struct_t *_this,float target, float kp, float ki, float kd)
{
    _this->_target=target;
    _this->_kp=kp;
    _this->_ki=ki;
    _this->_kd=kd;	
    _this->_past=millis();
    _this->_prevError=0;
}

//External method
void setPropotionality(PID_Struct_t *_this, float kp, float ki, float kd)
{
	_this->_kp=kp;
	_this->_ki=ki;
	_this->_kd=kd;
}
float returnTarget(PID_Struct_t *_this)
{
	return _this->_target;
}

void clearIntegral(PID_Struct_t *_this)
{
     _this->_integral=0;
}
void updateTarget(PID_Struct_t *_this, float target)
{
    _this->_target=target;
}
float readDerivative(PID_Struct_t *_this)
{
	return _this->_derivative;	
}
float readError(PID_Struct_t *_this)
{
	return _this->_error;
} 
float readIntegral(PID_Struct_t *_this)
 {
 	return _this->_integral;
 }

float readOutput(PID_Struct_t *_this)
 { 	
 	return _this->_output;
 }
 void clearSystem(PID_Struct_t *_this)
 {
 	_this->_error=0;
 	_this->_prevError=0;
 	_this->_derivative=0;
 	_this->_integral=0;
 	_this->_output=0;
 	_this->_past=millis(); 	
 }
//External method
int updateOutput(PID_Struct_t *_this, float sample)
{
	//update dt in seconds
	_this->_now=millis();
	_this->_dt=(_this->_now-_this->_past);
	_this->_past=_this->_now;
	
	//update error and dError

	_this->_error = _this->_target - sample;	
	_this->_dError = _this->_error - _this->_prevError;
    if(_this->_dt != 0)
    {
        _this->_derivative = _this->_dError/_this->_dt;

    }
	_this->_integral += _this->_error;  //*_dt;
	_this->_output=0;
	if(_this->_kp!=0)
    {
        _this->_output+=(_this->_error * _this->_kp);
    }
    if(_this->_kd != 0)
    {    
        _this->_output += (_this->_derivative * _this->_kd);
    }
    if(_this->_ki != 0)
    {
        _this->_output += (_this->_integral * _this->_ki);
    } 

    _this->_prevError = _this->_error;
    return _this->_output;
}

