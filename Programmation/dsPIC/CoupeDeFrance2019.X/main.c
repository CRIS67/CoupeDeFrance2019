/*
 * File:   main.c
 * Author: Quentin BOYER
 *
 * Created on September 18, 2017, 6:59 PM
 */

#include "constant.h"



#include <xc.h>
#include <math.h>
#include <p33EP512GM310.h>
#include <stdint.h>
#include "clock.h"
#include "GPIO.h"
#include "timer.h"
#include "PWM.h"
#include "QEI.h"
#include "PID.h"
#include "SPI.h"
#include "ADC.h"
#include "UART.h"
#include "interrupt.h"
#include "US.h"
#include "AX12.h"
#include "DMA.h"

void testAccMax();
void reglageDiametre();
void straightPath(double cx, double cy, double ct, double speedMax, double accMax);
void go(double cx, double cy, double speedMax, double accMax);
void goBack(double cx, double cy, double speedMax, double accMax);
void turn(double ct);
void turnNOLIMIT(double ct);

void modif_straightPath(double arg_cx, double arg_cy, double arg_ct, double arg_speedMax, double accMax);

void test();
void test2();
void testPWM();
void testSPI();
void testADC();
void testInterrupt();
void testDelay();
void setMotLin(uint8_t state);
double readAdcLowPass(uint8_t channel, uint16_t nbSamples, double coefLP);

//Global variables
//char TX[TX_SIZE];
char RX[RX_SIZE];
char unsigned TX_i;
char unsigned RX_i;
/*Current position*/
volatile long double x;
volatile long double y;
volatile long double theta;
/*Current setpoint position*/
volatile long double xc;
volatile long double yc;
volatile long double thetac;
/*Final setpoint position*/
volatile long double xf;
volatile long double yf;
volatile long double tf;

uint8_t finalPoint = 1;

volatile PID pidSpeedLeft, pidSpeedRight, pidDistance, pidAngle;
int state = 0;
int R,L;

//volatile char arrived;
//volatile char arrived_2;

extern volatile char   US_ON[NB_US];
extern volatile char   US_R[NB_US];
extern volatile double US[NB_US];

volatile char sendBT = 0;

extern volatile double receivedX,receivedY,receivedTheta;
extern volatile unsigned char newPosReceived;
extern volatile unsigned char newPosBackReceived;
extern volatile unsigned char newAngleReceived;
extern volatile unsigned char back;

extern volatile unsigned char debugPosRpi;

volatile unsigned char stop = 1;

unsigned char detectUS = 0;
unsigned char sensDetectUS = 1;

extern double var_ACC_MAX;

// <editor-fold defaultstate="collapsed" desc="Génération de trajectoire">
extern unsigned char statePathGeneration;
extern unsigned char stateTrap;

extern double cx, cy, ct;

extern double sign;

extern double phi;
extern double angle;

extern double theta0;
extern double angularVelocity;
extern double prevAngularVelocity;
extern double maxAngularVelocity;

extern double AngularAcceleration;

extern double angle1;


extern double dx;
extern double dy;
extern double alpha;

extern double totalDistance;
extern double dist;

extern double y_0;
extern double x_0;
extern double speed;
extern double precSpeed;

extern double acc;
extern double speedMax;

extern double dist1; // </editor-fold>

extern uint8_t BufferA[8];
extern volatile uint8_t RxDMABuffer[RX_DMA_SIZE];
extern uint16_t start;

extern uint16_t iD2,iF2;

volatile uint8_t verbose = 0;

volatile double funSpeed = 1000;
volatile double funAcc = 1000;

volatile double funAngularSpeed = 10;
volatile double funAngularAcc = 1;

int sens = 0;


volatile long double kahanErrorX;
volatile long double kahanErrorY;
volatile long double kahanErrorT;

volatile uint8_t arrived = 1;

int main(){
    int o;
    TRISFbits.TRISF7 = 0;
    LATFbits.LATF7 = 1;
    for (o = 0; o < 10000; o++);
    LATFbits.LATF7 = 0;
    for (o = 0; o < 10000; o++);
    LATFbits.LATF7 = 1;
    for (o = 0; o < 10000; o++);
    LATFbits.LATF7 = 0;
    for (o = 0; o < 10000; o++);
    LATFbits.LATF7 = 1;
    for (o = 0; o < 10000; o++);
    LATFbits.LATF7 = 0;
    for (o = 0; o < 10000; o++);
    
    initClock(); //Clock 140 MHz
    initGPIO();
    initPWM();
    initQEI();
    initDMA();
    initUART();
    initAX12();
    initInt();
    initUS();
    initADC();
    initSPI();
    
    
    x = 0;
    y = 0;
    theta = 0;
    
    xc = 0;
    yc = 0;
    thetac = 0;
    
    xf = 0;
    yf = 0;
    tf = 0;
    
    TX_i = 0;
    RX_i = 0;
    
    R = 0;
    L = 0;
    
    //double i;
    // <editor-fold defaultstate="collapsed" desc="Start">
    /*
     * LED_0 = 1;
    for (i = 0; i < 10000; i++);
    LED_0 = 0;
    for (i = 0; i < 10000; i++);
    LED_0 = 1;
    for (i = 0; i < 10000; i++);
    LED_0 = 0;
    for (i = 0; i < 10000; i++);
    LED_0 = 1;
    for (i = 0; i < 10000; i++);
    LED_0 = 0;
    for (i = 0; i < 10000; i++);
    LED_0 = 1;
    for (i = 0; i < 10000; i++);
    LED_0 = 0; */
    // </editor-fold>

    
    /*POS1CNTL = 0x8000;
    POS2CNTL = 0x8000;*/
    POS1CNTL = 0x0000;
    POS2CNTL = 0x0000;
    testSendToMotor(0, 0);
    initAllPID(&pidSpeedLeft, &pidSpeedRight, &pidDistance, &pidAngle);
    initTimer();
    //IEC0bits.T1IE = 0;  //stop asserv
    //sendToMotor(8,0);
    double oldCurrent = 0;
    double coefLP = 0.01;
    //testSendToMotor(2, 2);
    x = 1000;
    y = 1500;
    theta = PI;
    
    xc = x;
    yc = y;
    thetac = theta;
    
    xf = x;
    yf = y;
    tf = theta;
    while(1){
        /*double current;
        uint16_t cnt;
        for(cnt = 0; cnt < 1000;cnt++){
            int mes = readADC(ADC_CHANNEL_I_PUMP);
            current = mes;
            current = oldCurrent + (current - oldCurrent)*coefLP;
            oldCurrent = current;
            //delay_us(100);
        }
        
        if(current < 40){
            LED_0 = 1;
        }
        else{
            LED_0 = 0;
        }
        
        if(current < 50){
            LED_1 = 1;
        }
        else{
            LED_1 = 0;
        }
        
        if(current < 100){
            LED_2 = 1;
        }
        else{
            LED_2 = 0;
        }*/
        delay_ms(10);
        LED_PLATINE = !LED_PLATINE;
        CheckMessages();
        sendPos();
        if(newPosReceived){
                if(0){
                    
                    newPosReceived = 0;
                    
                    double angularVelocity = 0;
                    double maxAngularVelocity = funAngularSpeed;
                    double AngularAcceleration = funAngularAcc;
                    double angle = 0;
                    theta0 = theta;
                    double prevAngularVelocity = 0;
                    double phi = 20*PI;
                    double sign = 1;
                    if(sens == 0){
                        sign = -1;
                        sens = 1;
                    }
                    else{
                        sign = 1;
                        sens = 0;
                    }
                    finalPoint = 0; 
                            while(angularVelocity < maxAngularVelocity && angle < phi/2){
                                angularVelocity += AngularAcceleration * TE;
                                angle += TE * (prevAngularVelocity + angularVelocity) / 2;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                            double angle1 = angle;
                            while(angle < phi - angle1){
                                angle += TE * angularVelocity;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                            AngularAcceleration = -AngularAcceleration;
                            while(angularVelocity > 0 && angle < phi){
                                angularVelocity += AngularAcceleration * TE;
                                angle += TE * (prevAngularVelocity + angularVelocity) / 2;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                    finalPoint = 1;
                    
                }
                else{
                    statePathGeneration = 42;
                    delay_ms(100);
                    newPosReceived = 0;
                    if(newPosBackReceived)
                    {
                        newPosBackReceived = 0;
                        //printRpi(("DEBUG goBack main 1\n"));
                        goBack(receivedX,receivedY,2000,200);
                        //printRpi(("DEBUG goBack main 2\n"));
                    }
                    else{
                        //go(receivedX,receivedY,2000,200);
                        //go(receivedX,receivedY,1000,100);
                        modif_straightPath(receivedX,receivedY,0,funSpeed,funAcc);
                        //modif_straightPath(receivedX,receivedY,0,1000,100);
                    }
                }
            }
            if(newAngleReceived){
                if(0){
                    newAngleReceived = 0;
                    turn(receivedTheta);
                }
                else{
                    newAngleReceived = 0;
                    
                    double angularVelocity = 0;
                    double maxAngularVelocity = funAngularSpeed;
                    double AngularAcceleration = funAngularAcc;
                    double angle = 0;
                    theta0 = theta;
                    double prevAngularVelocity = 0;
                    double phi = 20*PI;
                    double sign = 1;
                    if(sens == 0){
                        sign = -1;
                        sens = 1;
                    }
                    else{
                        sign = 1;
                        sens = 0;
                    }
                    finalPoint = 0; 
                            while(angularVelocity < maxAngularVelocity && angle < phi/2){
                                angularVelocity += AngularAcceleration * TE;
                                angle += TE * (prevAngularVelocity + angularVelocity) / 2;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                            double angle1 = angle;
                            while(angle < phi - angle1){
                                angle += TE * angularVelocity;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                            AngularAcceleration = -AngularAcceleration;
                            while(angularVelocity > 0 && angle < phi){
                                angularVelocity += AngularAcceleration * TE;
                                angle += TE * (prevAngularVelocity + angularVelocity) / 2;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                    finalPoint = 1;
                }
                
            }
        //sendPosLongDouble();)
        //plot(1,(uint32_t)((int32_t)(current)));
    }
    
    initAllPID(&pidSpeedLeft, &pidSpeedRight, &pidDistance, &pidAngle);
    initTimer();
    state = 1;
    
    stop = 0;

    int m,p = 0;
    p++;
    x = 1000;
    y = 1500;
    theta = PI;
    
    /*x = 0;
    y = 0;
    theta = 0;*/
    
    xc = x;
    yc = y;
    thetac = theta;
    
    xf = x;
    yf = y;
    tf = theta;
    for(p = 0; p <= 6; p++)
        servoUs(p,0);
    verbose = 0;
    
    //uint32_t foo = 0;
    
    //TRISFbits.TRISF7 = 0;
    //uint16_t iDelay,jDelay;
    //LATFbits.LATF7 = 1;
    
    delay_ms(2000);
    while(0){
        delay_ms(500);
        CheckMessages();
        sendPosLongDouble();
        //long double *ptr = &d;
        //char *ptrCx = (char*)ptrKx = &kahanErrorX;
        //char *ptrCy = (char*)ptrKy = &kahanErrorY;
        //char *ptrCt = (char*)ptrKt = &kahanErrorT;
        /*sendLog("d[0] = ");
        sendLog(itoa(ptrC[0]));
        sendLog("\n");
        sendLog("d[1] = ");
        sendLog(itoa(ptrC[01]));
        sendLog("\n");
        sendLog("d[20] = ");
        sendLog(itoa(ptrC[2]));
        sendLog("\n");
        sendLog("d[3] = ");
        sendLog(itoa(ptrC[3]));
        sendLog("\n");*/
        //sendPos();
        /*sendLog("errorX = ");
        sendLog(dtoa((double)(kahanErrorX*1000000)));
        sendLog("\n");
        sendLog("errorY = ");
        sendLog(dtoa((double)(kahanErrorY*1000000)));
        sendLog("\n");
        sendLog("errorT = ");
        sendLog(dtoa((double)(kahanErrorT*1000000)));
        sendLog("\n");*/
    }
    while(0){
        CheckMessages();
        delay_ms(100);
    }
    while(0){
        xc = 250;
        xf = 250;
        CheckMessages();
        delay_ms(3000);
        xc = 0;
        xf = 0;
        CheckMessages();
        delay_ms(3000);
        xc = -250;
        xf = -250;
        CheckMessages();
        delay_ms(3000);
        xc = 0;
        xf = 0;
        CheckMessages();
        delay_ms(3000);
    }
    
    while(0){
        sendLog("x = ");
        sendLog(dtoa(x));
        sendLog(" | y = ");
        sendLog(dtoa(y));
        sendLog(" | t = ");
        sendLog(dtoa(theta*180/PI));
        sendLog("\n");
        sendPos();
        
        CheckMessages();
        delay_ms(500);
    }
    while(0){
        //uint16_t dummy = SPI2BUF;
        sendLog("0x20 (debug) sent to ComeCard\n");
        SPI2BUF = 0x20; //debug
        delay_ms(500);
        //dummy = SPI2BUF;
        //CheckMessages();
        //sendLog(itoa(dummy));
        sendLog("0x21 (autre) sent to ComeCard\n");
        SPI2BUF = 0x21; //autre
        delay_ms(500);
        //dummy = SPI2BUF;
        CheckMessages();
        //sendLog(itoa(dummy));
    }
    while(0){
        /*for(p = 10; p < 50; p++){
            plot(p,100);
            delay_ms(10);
        }*/
        /*plot(1,(uint32_t)(int32_t)(thetac*1000));
        //plot(2,(uint32_t)(int32_t)(angularVelocity*1000));
        //plot(3,(uint32_t)(int32_t)(angle*1000));
        plot(4,(uint32_t)(int32_t)(theta*1000));
        plot(5,(uint32_t)(int32_t)(pidAngle.setPoint*1000));*/
        CheckMessages();
        //if(!stop){
            delay_ms(500);
            //plot(1,micros());
            sendPos();
            //sendPosLongDouble();
            //sendRupt();
            //sendUS();
            /*plot(1,iF2);
            plot(2,iD2);*/
            m++;
            if(m >= 10){
                m = 0;
                //sendLog("m overflow\n");
            }
            //send("timing\n",strlen("timing\n"));
            if(newPosReceived){
                if(0){
                    
                    newPosReceived = 0;
                    
                    double angularVelocity = 0;
                    double maxAngularVelocity = funAngularSpeed;
                    double AngularAcceleration = funAngularAcc;
                    double angle = 0;
                    theta0 = theta;
                    double prevAngularVelocity = 0;
                    double phi = 20*PI;
                    double sign = 1;
                    if(sens == 0){
                        sign = -1;
                        sens = 1;
                    }
                    else{
                        sign = 1;
                        sens = 0;
                    }
                    finalPoint = 0; 
                            while(angularVelocity < maxAngularVelocity && angle < phi/2){
                                angularVelocity += AngularAcceleration * TE;
                                angle += TE * (prevAngularVelocity + angularVelocity) / 2;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                            double angle1 = angle;
                            while(angle < phi - angle1){
                                angle += TE * angularVelocity;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                            AngularAcceleration = -AngularAcceleration;
                            while(angularVelocity > 0 && angle < phi){
                                angularVelocity += AngularAcceleration * TE;
                                angle += TE * (prevAngularVelocity + angularVelocity) / 2;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                    finalPoint = 1;
                    
                }
                else{
                    statePathGeneration = 42;
                    delay_ms(100);
                    newPosReceived = 0;
                    if(newPosBackReceived)
                    {
                        newPosBackReceived = 0;
                        //printRpi(("DEBUG goBack main 1\n"));
                        goBack(receivedX,receivedY,2000,200);
                        //printRpi(("DEBUG goBack main 2\n"));
                    }
                    else{
                        //go(receivedX,receivedY,2000,200);
                        //go(receivedX,receivedY,1000,100);
                        modif_straightPath(receivedX,receivedY,0,funSpeed,funAcc);
                        //modif_straightPath(receivedX,receivedY,0,1000,100);
                    }
                }
            }
            if(newAngleReceived){
                if(0){
                    newAngleReceived = 0;
                    turn(receivedTheta);
                }
                else{
                    newAngleReceived = 0;
                    
                    double angularVelocity = 0;
                    double maxAngularVelocity = funAngularSpeed;
                    double AngularAcceleration = funAngularAcc;
                    double angle = 0;
                    theta0 = theta;
                    double prevAngularVelocity = 0;
                    double phi = 20*PI;
                    double sign = 1;
                    if(sens == 0){
                        sign = -1;
                        sens = 1;
                    }
                    else{
                        sign = 1;
                        sens = 0;
                    }
                    finalPoint = 0; 
                            while(angularVelocity < maxAngularVelocity && angle < phi/2){
                                angularVelocity += AngularAcceleration * TE;
                                angle += TE * (prevAngularVelocity + angularVelocity) / 2;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                            double angle1 = angle;
                            while(angle < phi - angle1){
                                angle += TE * angularVelocity;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                            AngularAcceleration = -AngularAcceleration;
                            while(angularVelocity > 0 && angle < phi){
                                angularVelocity += AngularAcceleration * TE;
                                angle += TE * (prevAngularVelocity + angularVelocity) / 2;
                                thetac = theta0 + angle * sign;
                                prevAngularVelocity = angularVelocity;
                                delay_ms(TE * 1000);
                            }
                    finalPoint = 1;
                }
                
            }
            /*if(statePathGeneration){
                plot(1,(uint32)(int32)(((thetac*3600)/(2*PI))));
                plot(2,(uint32)(int32)(((angularVelocity*3600)/(2*PI))));
                plot(3,(uint32)(int32)(((theta*3600)/(2*PI))));
            }*/
        //}
        
    }

    while(1){
        if(stop){
            //IEC0bits.T1IE = 0;  //stop asserv
            sendToMotor(0,0);   //stop motors
            while(stop){        //wait for start signal
            }
            //IEC0bits.T1IE = 1;  //restart asserv
        }
        switch(state){
            case 0: //init
                IEC0bits.T1IE = 0;
                x = 0;
                y = 0;
                theta = 0;
                xc = 0;
                yc = 0;
                thetac = 0;
                xf = 0;
                yf = 0;
                tf = 0;
                POS1CNTL = 0x8000;
                POS2CNTL = 0x8000;
                IEC0bits.T1IE = 1;
                delay_ms(1000);
                state++;
                break;
            case 1 :
                if(newPosReceived){
                    newPosReceived = 0;
                    if(newPosBackReceived)
                    {
                        newPosBackReceived = 0;
                        //printRpi(("DEBUG goBack main 1\n"));
                        goBack(receivedX,receivedY,2000,200);
                        //printRpi(("DEBUG goBack main 2\n"));
                    }
                    else
                        //go(receivedX,receivedY,2000,200);
                        modif_straightPath(receivedX,receivedY,0,1000,1000);
                }
                if(newAngleReceived){
                    newAngleReceived = 0;
                    turn(receivedTheta);
                }
                delay_ms(100);
                break;
            default:
                sendLog("ERROR UNDEFINED STATE");
                break;       
        }
       
    }
    return 0;
}

void testAccMax(){
    double commandeL = 12;
    double prevCommandeL = 0;
    double delta = 0.1;
    double newCL;
    while(1){
        if(commandeL - prevCommandeL > delta)
            newCL = prevCommandeL + delta;
        else if(commandeL - prevCommandeL < -delta)
            newCL = prevCommandeL - delta;
        else
            newCL = commandeL;
        prevCommandeL = newCL;
        sendToMotor(0,newCL);
        delay_ms(30);
    }
}
void reglageDiametre(){
    IEC0bits.T1IE = 0;
    POS1CNTL = 0x8000;
    POS2CNTL = 0x8000;
    double l,r,l2,r2;
    double D1,D2;
    while(1){
        l = (double)(POS1CNTL - 0x8000) * PI * 2 * ENCODER_WHEEL_RADIUS / 4096;
        r = (POS2CNTL - 0x8000) * PI * 2 * ENCODER_WHEEL_RADIUS / 4096;
        l2 = (double)(POS1CNTL - 0x8000) * PI * 2 / 4096;
        r2 = (double)(POS2CNTL - 0x8000) * PI * 2 / 4096;
        D1 = 300/l2;
        D2 = 300/r2;
        
        /*print("l : ");
        print(itoa((int)l));
        print("      r : ");
        print(itoa((int)r));
        print("      D1 : ");
        print(itoa((int)(D1)));
        print("      D1*100 : ");
        print(itoa((int)(D1*100)));
        print("      D2 : ");
        print(itoa((int)(D2)));
        print("      D2*100 : ");
        print(itoa((int)(D2*100)));
       
        print("\r\n");*/
        delay_ms(200);
    }   
}
void straightPath(double cx, double cy, double ct, double speedMax, double accMax){
    double i;
    double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = thetaRobotPoint - theta;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 10;
    double maxAngularVelocity = 10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0)
        sign = 1;
    else
        sign = -1;
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    delay_ms(500);
    /* Phase 2 : straight line */
    xf = cx;
    yf = cy;
    tf = ct;
    if(speedMax < 0)
        speedMax = 0;
    else if(speedMax > SPEED_MAX)
        speedMax = SPEED_MAX;
    if(accMax < 0)
        accMax = 0;
    else if(accMax > ACCELERATION_MAX)
        accMax = ACCELERATION_MAX;
    
    double y0 = y;
	double x0 = x;
	double dx = cx - x0;
	double dy = cy - y0;
	double alpha = atan2(dy,dx);
	double totalDistance = sqrt(dx*dx+dy*dy);
    
    double acc = accMax;
	double speed = 0;
	double precSpeed = 0;
	double dist = 0;
	while(speed < speedMax && dist < totalDistance/2){
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    double dist1 = dist;
	//2
	if(speed > speedMax)
        speed = speedMax;
	while(dist < totalDistance - dist1){               //Condition
		dist += TE * speed;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
	//3
	acc = -acc;
	while(speed > 0 && dist < totalDistance){				//Condition		//v > 0			/		d < totalDistance
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    xc = cx;
    yc = cy;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    delay_ms(500);
    //while(!arrived);
    
    /*Phase 3 : rotation */
    if(ct > theta){
        for(i = theta; i <= ct; i+= ROTATION_SPEED){
            thetac = i;
            delay_ms(DELAY_SPEED);
        }
    }
    else{
        for(i = theta; i >= ct; i-= ROTATION_SPEED){
            thetac = i;
            delay_ms(DELAY_SPEED);
        }
    }
    //print("Phase 3 OK\n");
    thetac = ct;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    delay_ms(500);
    //while(!arrived);
}

void go(double cx, double cy, double speedMax, double accMax){
    //arrived_2 = 0;
    double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = thetaRobotPoint - theta;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 3;//1;//10; envoie du steak
    double maxAngularVelocity = 3;//1;//10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0){
        sign = 1;
    }
    else{
        sign = -1;
        phi = -phi;
    }
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    //delay_ms(100);
    /* Phase 2 : straight line */
    xf = cx;
    yf = cy;
    if(speedMax < 0)
        speedMax = 0;
    else if(speedMax > SPEED_MAX)
        speedMax = SPEED_MAX;
    if(accMax < 0)
        accMax = 0;
    else if(accMax > ACCELERATION_MAX)
        accMax = ACCELERATION_MAX;
    
    double y0 = y;
	double x0 = x;
	double dx = cx - x0;
	double dy = cy - y0;
	double alpha = atan2(dy,dx);
	double totalDistance = sqrt(dx*dx+dy*dy);
    
    double acc = accMax;
	double speed = 0;
	double precSpeed = 0;
	double dist = 0;
	while(speed < speedMax && dist < totalDistance/2){
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    double dist1 = dist;
	//2
	if(speed > speedMax)
        speed = speedMax;
	while(dist < totalDistance - dist1){               //Condition
		dist += TE * speed;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
	//3
	acc = -acc;
	while(speed > 0 && dist < totalDistance){				//Condition		//v > 0			/		d < totalDistance
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    xc = cx;
    yc = cy;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    //delay_ms(100);
    //while(!arrived);
}

void goBack(double cx, double cy, double speedMax, double accMax){
    //arrived_2 = 0;
    //printRpi("DEBUG begin of goBack()");
    back = 1;
    double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = thetaRobotPoint - theta - PI;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 3;//10; envoie du steak
    double maxAngularVelocity = 3;//10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0){
        sign = 1;
    }
    else{
        sign = -1;
        phi = -phi;
    }
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    //delay_ms(500);
    /* Phase 2 : straight line */
    xf = cx;
    yf = cy;
    if(speedMax < 0)
        speedMax = 0;
    else if(speedMax > SPEED_MAX)
        speedMax = SPEED_MAX;
    if(accMax < 0)
        accMax = 0;
    else if(accMax > ACCELERATION_MAX)
        accMax = ACCELERATION_MAX;
    
    double y0 = y;
	double x0 = x;
	double dx = cx - x0;
	double dy = cy - y0;
	double alpha = atan2(dy,dx);
	double totalDistance = sqrt(dx*dx+dy*dy);
    
    double acc = accMax;
	double speed = 0;
	double precSpeed = 0;
	double dist = 0;
	while(speed < speedMax && dist < totalDistance/2){
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    double dist1 = dist;
	//2
	if(speed > speedMax)
        speed = speedMax;
	while(dist < totalDistance - dist1){               //Condition
		dist += TE * speed;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
	//3
	acc = -acc;
	while(speed > 0 && dist < totalDistance){				//Condition		//v > 0			/		d < totalDistance
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    xc = cx;
    yc = cy;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    //delay_ms(500);
    back = 0;
    //printRpi("DEBUG end of goBack()\n");
    //while(!arrived);
}

void turn(double ct){
    //double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = ct - theta;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 3;//1;//10; envoie du steak
    double maxAngularVelocity = 3;//1;//10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0){
        sign = 1;
    }
    else{
        sign = -1;
        phi = -phi;
    }
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    delay_ms(500);
    
}
void modif_straightPath(double arg_cx, double arg_cy, double arg_ct, double arg_speedMax, double accMax){
    /*INIT*/
    
    cx = arg_cx;
    cy = arg_cy;
    ct = arg_ct;
    
    speedMax = arg_speedMax;
    
    double thetaRobotPoint = atan2(cy-y,cx-x);
    phi = thetaRobotPoint - theta;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    finalPoint = 0;
    theta0 = theta;
    AngularAcceleration = funAngularAcc;
    maxAngularVelocity = funAngularSpeed;
    /*AngularAcceleration = 1;
    maxAngularVelocity = 10;*/
    angularVelocity = 0;
    prevAngularVelocity = 0;
    angle = 0;
    
    if(phi > 0)
        sign = 1;
    else{
        sign = -1;
        phi = -phi;
    }
    
    if(speedMax < 0)
        speedMax = 0;
    else if(speedMax > SPEED_MAX)
        speedMax = SPEED_MAX;
    if(accMax < 0)
        accMax = 0;
    else if(accMax > ACCELERATION_MAX)
        accMax = ACCELERATION_MAX;
    
    acc = accMax;
	speed = 0;
	precSpeed = 0;
	dist = 0;

    statePathGeneration = 1;
    stateTrap = 1;
}
void turnNOLIMIT(double ct){
    //double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = ct - theta;
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 3;//1;//10; envoie du steak
    double maxAngularVelocity = 10;//1;//10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0){
        sign = 1;
    }
    else{
        sign = -1;
        phi = -phi;
    }
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    delay_ms(500);
    
}
void testPIDs(){
    while(1){
        
        x = 0;
        y = 0;
        theta = 0;
        xc = x;
        yc = y;
        thetac = theta;
        xf = x;
        yf = y;
        tf = theta;
        
        while(stop);
        
        modif_straightPath(300,300,0,1000,100);
        
    }
}
void setMotLin(uint8_t state){
    double acc = 5;     //v/s^1
    double speed = 15;   //v
    double dt = 0.01;   //s
    
    double voltage = 0;
    
    unsigned long t1 = millis();
    if(state == 0){ //out
        while(!RUPT_ACT_0){ //wait for rupt
            if( ( (millis() - t1) > (dt*1000) ) && (voltage < speed) ){
                voltage += acc*dt;
                if(voltage > speed){
                    voltage = speed;
                }
                motorVoltage(ID_MOTOR_LINEAR,voltage);
            }
        }
        motorVoltage(ID_MOTOR_LINEAR,-voltage); //brake
        delay_ms(80);
        motorVoltage(ID_MOTOR_LINEAR,0);    //stop
    }
    else if(state == 1){ //in
        while(!RUPT_ACT_1){ //wait for rupt
            if( ( (millis() - t1) > (dt*1000) ) && (voltage > -speed) ){
                voltage -= acc*dt;
                if(voltage < -speed){
                    voltage = -speed;
                }
                motorVoltage(ID_MOTOR_LINEAR,voltage);
            }
        }
        motorVoltage(ID_MOTOR_LINEAR,-voltage); //brake
        delay_ms(80);
        motorVoltage(ID_MOTOR_LINEAR,0);    //stop
    }
    
}
double readAdcLowPass(uint8_t channel, uint16_t nbSamples, double coefLP){
	double oldCurrent = 0;
    double current;
	uint16_t cnt;
	for(cnt = 0; cnt < nbSamples;cnt++){
		int mes = readADC(channel);
		current = mes;
		current = oldCurrent + (current - oldCurrent)*coefLP;
		oldCurrent = current;
	}
	return current;
}