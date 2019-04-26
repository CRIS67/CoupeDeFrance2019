#include "SPI.hpp"

SPI::SPI(int channel, int speed){
	m_channel = channel;
    m_fd = wiringPiSPISetup(channel, speed);
	pinMode(PIN_MUX_A, OUTPUT);
	pinMode(PIN_MUX_B, OUTPUT);
}
SPI::~SPI(){

}
/*Change slave select line*/
void SPI::changeSlave(uint8_t id){
	digitalWrite(PIN_MUX_A,(id & 0x1));			//bit0 of id -> MUX_A
	digitalWrite(PIN_MUX_B,((id >> 1) & 0x1));	//bit1 of id -> MUX_B
}
/*Lock the SPI bus from preventing multiple acces from other threads*/
void SPI::lock(){
	m_mutex.lock();
}
/*Unlock the SPI bus*/
void SPI::unlock(){
	m_mutex.unlock();
}
int SPI::getFd const(){
	return m_fd;
}
int SPI::getChannel const(){
	return m_channel;
}