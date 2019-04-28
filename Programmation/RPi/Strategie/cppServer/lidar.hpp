#ifndef LIDAR_H
#define LIDAR_H

#define SPI_FREQUENCY	500000
#define CHANNEL 0

#define SPI_DELAY		100

#define SIZE_BUFFER_RX	1000

#define CODE_VAR_DISTANCE   1
#define CODE_VAR_ANGLE      2

#define LIDAR_CMD_DEBUG               1
#define LIDAR_CMD_START               2
#define LIDAR_CMD_STOP                3
#define LIDAR_CMD_GET_DISTANCE        4
#define LIDAR_CMD_GET_ANGLE           5
#define LIDAR_CMD_GET_STARTBIT        6
#define LIDAR_CMD_GET_QUALITY         7
#define LIDAR_CMD_SET_AUTOSTART       8
#define LIDAR_CMD_RESET_AUTOSTART     9

#define LIDAR_CMD_GET_DATA_AVAILABLE    10
#define LIDAR_CMD_GET_DETECTED_POINTS   11
#define LIDAR_CMD_GET_RAW_POINT         12

#define LIDAR_CMD_SET_SPEED             20
#define LIDAR_CMD_GET_SPEED             21


#define LIDAR_RET_DEBUG_DEBUG       42 
#define LIDAR_RET_DEBUG_START       43 
#define LIDAR_RET_DEBUG_STOP        44
#define LIDAR_RET_DATA_AVAILABLE    100
#define LIDAR_RET_DETECTED_POINTS   101
#define LIDAR_RET_RAW_POINT         102

#define LIDAR_RET_SPEED             121

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <queue>
#include <unistd.h>
#include <iostream>

class Lidar
{
    public:
        Lidar();
        virtual ~Lidar();
		
		void start();
		void stop();
		
		void sendReceiveSPI(uint8_t data);
		void flush();
		void checkMessages();
		void sendSPI(uint8_t *buf, uint8_t bufSize);
		void getAvailableData();
		void getRawPoint();
		void getDetectedPoints();
		void setSpeed(uint8_t speed);
		void getSpeed();
		
		int16_t x = 1500,y = 1000, t = 45;
		uint8_t bufferRx[SIZE_BUFFER_RX];
		uint8_t iRxIn = 0;
		uint8_t iRxOut = 0;
		bool receivingMsg = false;
		uint8_t currentMsgSize = 0;
		uint8_t nbBytesReceived = 0;
		uint32_t nbBytesReceivedTotal = 0;
		uint8_t nbMsgReceived = 0;
    protected:
		int fd;
    private:
};

#endif // LIDAR_H
