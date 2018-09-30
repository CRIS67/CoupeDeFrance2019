/* 
 * File: ADC.h  
 * Author: Quentin BOYER
 * Comments: 
 * Revision history: 1
 */

#ifndef ADC_H
#define	ADC_H

#include <xc.h> // include processor files - each processor file is guarded.
#include "constant.h"

void initADC();
void initADC1();
int readADC1();


#endif	/* ADC_H */