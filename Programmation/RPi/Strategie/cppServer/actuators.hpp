#ifndef ACTUATORS_H
#define ACTUATORS_H

#define SPI_DELAY	100		//100µs
#define CHANNEL 0

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>
#include <unistd.h>

#include <iostream>
/*#include <iostream>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <unistd.h>
#include <math.h>
#include <string.h>*/
class Actuators
{
    public:
        Actuators(int fd_SPI);
        virtual ~Actuators();
		void MoveServoExtr(int nb_bras, int pos);
		void MoveServo(int nb_bras, int pos);
		void SetPump(int nb_bras, int state);
		void GetColor(int nb_bras);
		void GetCurrent(int nb_bras);
		void GetCurrentFull(int nb_bras);
		/*void SetPrgm(void);
		void SetPos(void);
		void SetTxt(void);*/
		void Launchtest();
		void flush(uint16_t nb);
		int DebugGetCurrent(int nb_bras);
    protected:
    private:
		int m_fd;	//SPI file descriptor
};

#endif // ACTUATORS_H
