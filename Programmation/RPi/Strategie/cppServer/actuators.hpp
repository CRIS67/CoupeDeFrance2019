#ifndef ACTUATORS_H
#define ACTUATORS_H

#define SPI_DELAY	100		//100µs
#define CHANNEL 0

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>
#include <unistd.h>
#include <iostream>

#include "SPI.hpp"

class Actuators
{
    public:
        Actuators(SPI *pSpi,uint8_t id);
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
		int debugGetCurrentFull(int nb_bras);
    protected:
    private:
		uint8_t m_id;	//id of this SPI slave
		//int m_fd;		//SPI file descriptor
		SPI *m_pSpi;	//pointer to SPI instance
};

#endif // ACTUATORS_H
