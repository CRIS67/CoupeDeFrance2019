/* 
 * File: interrupt.h  
 * Author: Quentin BOYER
 * Comments: check most appropriate variable type <-> speed of execution 
 *              optimization : remove prevProcessVariable
 * Revision history: 1
 */

#ifndef INTERRUPT_H
#define	INTERRUPT_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "constant.h"
#define _ISR_PSV        __attribute__((__interrupt__, __auto_psv__))
//#define _ISR_PSV_FAST   __attribute__((interrupt(auto_psv, shadow)))
typedef enum pinCN pinCN;
enum pinCN
{
    DEFAULT,RUPT_1, RUPT_2, ECHO, ECHO_0, ECHO_1, ECHO_2, ECHO_3, ECHO_4, ECHO_5
};
/*typedef struct SavePort SavePort;
struct SavePort
{
    unsigned int b0 : 1;
    unsigned int b1 : 1;
    unsigned int b2 : 1;
    unsigned int b3 : 1;
    unsigned int b4 : 1;
    unsigned int b5 : 1;
    unsigned int b6 : 1;
    unsigned int b7 : 1;
    unsigned int b8 : 1;
    unsigned int b9 : 1;
    unsigned int b10 : 1;
    unsigned int b11 : 1;
    unsigned int b12 : 1;
    unsigned int b13 : 1;
    unsigned int b14 : 1;
    unsigned int b15 : 1;
};*/
void initInt();


#endif	/* INTERRUPT_H */