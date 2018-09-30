/*
 * File:   PID.c
 * Author: Quentin BOYER
 *
 * Created on 27 octobre 2017, 18:52
 * 
 * Notes : à tester
 * 
 */

#include "PID.h"

void initAllPID(volatile PID *pidSpeedLeft, volatile PID *pidSpeedRight, volatile PID *pidDistance, volatile PID *pidAngle){
    initPID(pidSpeedLeft,KP_SPEED_LEFT,KI_SPEED_LEFT,KD_SPEED_LEFT,BIAS_SPEED_LEFT,0,T_SPEED_LEFT,0,0);
    initPID(pidSpeedRight,KP_SPEED_RIGHT,KI_SPEED_RIGHT,KD_SPEED_RIGHT,BIAS_SPEED_RIGHT,0,T_SPEED_RIGHT,0,0);
    initPID(pidDistance,KP_DISTANCE,KI_DISTANCE,KD_DISTANCE,BIAS_DISTANCE,0,T_DISTANCE,0,0);
    initPID(pidAngle,KP_ANGLE,KI_ANGLE,KD_ANGLE,BIAS_ANGLE,0,T_ANGLE,0,0);
}

void initPID(volatile PID *pid, double Kp, double Ki, double Kd,double bias, double output, double period, double processVariable, double setPoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->bias = bias; 
    pid->output = output;
    pid->period = period;
    //pid->prevProcessVariable = processVariable;
    pid->processVariable = processVariable;
    pid->setPoint = setPoint;
    pid->sumI = 0;
    pid->prevError = 0;
    pid->prevSmoothError = 0;
    return;
}

void setSetPoint(volatile PID *pid, double setPoint){    //~useless
    pid->setPoint = setPoint;
    //pid->sumI = 0;
}

void setProcessValue(volatile PID *pid, double processVariable){ //~useless
    pid->processVariable = processVariable;
}

double compute(volatile PID *pid, double processVariable){
    
    int i;
    double smoothError;
    
    pid->processVariable = processVariable;
    double error = pid->setPoint - pid->processVariable;
    
    //double deriv = ((error - pid->prevError)/pid->period);
    for(i = 1; i < N_SMOOTHING; i++){
        smoothError = pid->prevSmoothError + SMOOTHING_FACTOR * (error - pid->prevSmoothError);
    }
    double deriv = ((smoothError - pid->prevSmoothError)/pid->period);
    
    pid->sumI = pid->sumI + error * pid->period;
    
    if(pid->sumI > MAX_I)
        pid->sumI = MAX_I;
    else if(pid->sumI < -MAX_I)
        pid->sumI = -MAX_I;
    
    pid->output = pid->bias + pid->Kp * error + pid->Ki * pid->sumI + pid->Kd * deriv;
            
    pid->prevError = error;
    pid->prevSmoothError = smoothError;
    
    return pid->output;
}
double modulo2Pi(double t){
    while(t > PI)
        t-= PI;
    while(t < -PI)
        t+= PI;
    return t;            
}
