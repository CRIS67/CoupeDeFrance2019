/* 
 * File: PWM.h  
 * Author: Quentin BOYER
 * Comments: 
 * Revision history: 1
 */

#ifndef PWM_H
#define	PWM_H

#include <xc.h> // include processor files - each processor file is guarded. 
#include "timer.h"
#include "constant.h"

/*#define sendToMotor(valueR, valueL) if(valueL > 0){\
        valueL += DEAD_ZONE;\
    }\
    else if(valueL < 0){\
        valueL -= DEAD_ZONE;\
    }\
    if(valueR > 0){\
        valueR += DEAD_ZONE;\
    }\
    else if(valueR < 0){\
        valueR -= DEAD_ZONE;\
    }\
    if(valueL <= -VSAT){\
        SENS_L = BACKWARD;\
        PWM_L = PWM_PR_L * VSAT / VBAT;\
    }\
    else if(valueL >= VSAT){\
        SENS_L = FORWARD;\
        PWM_L = PWM_PR_L * VSAT / VBAT;\
    }\
    else if(valueL < -DEAD_ZONE){\
        SENS_L = BACKWARD;\
        PWM_L = -(int)(valueL * PWM_PR_L / VBAT);\
    }\
    else if(valueL > DEAD_ZONE){\
        SENS_L = FORWARD;\
        PWM_L = (int)(valueL * PWM_PR_L / VBAT);\
    }\
    else{    \
        PWM_L = 0;\
    }\
    if(valueR <= -VSAT){\
        SENS_R = FORWARD;\
        PWM_R = PWM_PR_R * VSAT / VBAT;\
    }\
    else if(valueR >= VSAT){\
        SENS_R = BACKWARD;\
        PWM_R = PWM_PR_R * VSAT / VBAT;\
    }\
    else if(valueR < -DEAD_ZONE){\
        SENS_R = FORWARD;\
        PWM_R = -(int)(valueR * PWM_PR_R / VBAT);\
    }\
    else if(valueR > DEAD_ZONE){\
        SENS_R = BACKWARD;\
        PWM_R = (int)(valueR * PWM_PR_R / VBAT);\
    }\
    else{    \
        PWM_R = 0;\
    }*/

void initPWM();
void initPWM1();
void initPWM2();
void initPWM3();
void initPWM4();
void initPWM5();
void initPWM6();

void testServo();
void servoUs(unsigned char servo, double value);

void sendToMotor(double valueR, double valueL);

void testSendToMotor(double valueR, double valueL);


#endif	/* PWM_H */