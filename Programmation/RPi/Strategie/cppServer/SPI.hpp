#ifndef SPI_H
#define SPI_H

#define PIN_MUX_A	28		//pin 38 / BCM 20 / WiringPi 28
#define PIN_MUX_B	29		//pin 40 / BCM 21 / WiringPi 29

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>
#include <unistd.h>
#include <iostream>
#include <mutex>

class SPI
{
    public:
        SPI(int channel, int speed);
        virtual ~SPI();
		void changeSlave(uint8_t id);
		void lock();
		void unlock();
		int getFd();
		int getChannel();
    protected:
    private:
		int m_fd;			//SPI file descriptor
		int m_channel;		//SPI channel
		std::mutex m_mutex;	//SPI file descriptor
};

#endif // SPI_H
