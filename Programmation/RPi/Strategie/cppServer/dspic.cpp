#include "dspic.hpp"

DsPIC::DsPIC(){
    fd = serialOpen ("/dev/serial0", BAUDRATE);
}
DsPIC::~DsPIC(){

}
void DsPIC::servo(uint8_t id, uint16_t value){
    uint8_t buffer[RX_SIZE_SERVO + 1];
    buffer[0] = RX_SIZE_SERVO;
    buffer[1] = RX_CODE_SERVO;
    buffer[2] = id;
    buffer[3] = (uint8_t)(value >> 8);
    buffer[4] = (uint8_t)(value & 0xFF);
    buffer[5] = 0;
    for(int i = 0; i < RX_SIZE_SERVO; i++){
        buffer[5] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_SERVO + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::AX12(uint8_t id, uint16_t value){
    uint8_t buffer[RX_SIZE_AX12 + 1];
    buffer[0] = RX_SIZE_AX12;
    buffer[1] = RX_CODE_AX12;
    buffer[2] = id;
    buffer[3] = (uint8_t)(value >> 8);
    buffer[4] = (uint8_t)(value & 0xFF);
    buffer[5] = 0;
    for(int i = 0; i < RX_SIZE_AX12; i++){
        buffer[5] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_AX12 + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::motor(uint8_t id, int8_t value){
    uint8_t buffer[RX_SIZE_MOTOR + 1];
    buffer[0] = RX_SIZE_MOTOR;
    buffer[1] = RX_CODE_MOTOR;
    buffer[2] = id;
    buffer[3] = value;
    buffer[4] = 0;
    for(int i = 0; i < RX_SIZE_MOTOR; i++){
        buffer[4] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_MOTOR + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::start(){
    uint8_t buffer[RX_SIZE_START + 1];
    buffer[0] = RX_SIZE_START;
    buffer[1] = RX_CODE_START;
    buffer[2] = 0;
    for(int i = 0; i < RX_SIZE_START; i++){
        buffer[2] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_START + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::stop(){
    uint8_t buffer[RX_SIZE_STOP + 1];
    buffer[0] = RX_SIZE_STOP;
    buffer[1] = RX_CODE_STOP;
    buffer[2] = 0;
    for(int i = 0; i < RX_SIZE_STOP; i++){
        buffer[2] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_STOP + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::go(int16_t x, int16_t y,unsigned char rev, unsigned char relative){
    uint8_t option = 0;
    if(rev){
        option += MASK_OPTION_REVERSE;
    }
    if(relative){
        option += MASK_OPTION_RELATIVE;
    }
    uint8_t buffer[RX_SIZE_GO + 1];
    buffer[0] = RX_SIZE_GO;
    buffer[1] = RX_CODE_GO;
    buffer[2] = option;
    buffer[3] = (uint8_t)(x >> 8);
    buffer[4] = (uint8_t)(x & 0xFF);
    buffer[5] = (uint8_t)(y >> 8);
    buffer[6] = (uint8_t)(y & 0xFF);
    buffer[7] = 0;
    for(int i = 0; i < RX_SIZE_GO; i++){
        buffer[7] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_GO + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::turn(int16_t t,unsigned char rev, unsigned char relative){
    uint8_t option = 0;
    if(rev){
        option += MASK_OPTION_REVERSE;
    }
    if(relative){
        option += MASK_OPTION_RELATIVE;
    }
    uint8_t buffer[RX_SIZE_TURN + 1];
    buffer[0] = RX_SIZE_TURN;
    buffer[1] = RX_CODE_TURN;
    buffer[2] = option;
    buffer[3] = (uint8_t)(t >> 8);
    buffer[4] = (uint8_t)(t & 0xFF);
    buffer[5] = 0;
    for(int i = 0; i < RX_SIZE_TURN; i++){
        buffer[5] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_TURN + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::setVar32(uint8_t varCode, uint32_t var){

    uint8_t buffer[RX_SIZE_SET_32b + 1];
    buffer[0] = RX_SIZE_SET_32b;
    buffer[1] = RX_CODE_SET;
    buffer[2] = varCode;
    buffer[3] = VAR_32b;
    buffer[4] = (uint8_t)(var >> 24);
    buffer[5] = (uint8_t)(var >> 16);
    buffer[6] = (uint8_t)(var >> 8);
    buffer[7] = (uint8_t)(var & 0xFF);
    buffer[8] = 0;
    for(int i = 0; i < RX_SIZE_SET_32b; i++){
        buffer[8] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_SET_32b + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::setVar8(uint8_t varCode, uint8_t var){

    uint8_t buffer[RX_SIZE_SET_8b + 1];
    buffer[0] = RX_SIZE_SET_8b;
    buffer[1] = RX_CODE_SET;
    buffer[2] = varCode;
    buffer[3] = VAR_8b;
    buffer[4] = var;
    buffer[5] = 0;
    for(int i = 0; i < RX_SIZE_SET_8b; i++){
        buffer[5] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_SET_8b + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
void DsPIC::loadPID(){
    uint8_t buffer[RX_SIZE_GET + 1];
    buffer[0] = RX_SIZE_GET;
    buffer[1] = RX_CODE_GET;
    buffer[2] = CODE_VAR_ALLPID;
    buffer[3] = 0;
    for(int i = 0; i < RX_SIZE_GET; i++){
        buffer[3] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_GET + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
std::string DsPIC::async_read(){
    std::string s("");
    while (serialDataAvail(fd)){
      s += serialGetchar(fd);
    }
    return s;
}
 std::vector<uint8_t> DsPIC::readMsg(){
	//double delayUs = 1000000 / BAUDRATE;	// T = 1/f en µs	(0.5Mbaud => 2µs)
	
	int foo = serialGetchar(fd);
 	while(foo == -1){	//no reception during 10 sec
 		//delayMicroseconds(delayUs);
 		foo = serialGetchar(fd);
 	}
 	uint8_t RxSize = foo;
    std::vector<uint8_t> RxBuf;
    RxBuf.push_back(RxSize);

	for(int i = 0; i < RxSize; i++){
    	foo = serialGetchar(fd);
    	while(foo == -1){
    		//delayMicroseconds(delayUs);
 			foo = serialGetchar(fd);
 		}
        RxBuf.push_back(foo);
        //delayMicroseconds(delayUs);
        //delayMicroseconds(5);
    }


	/*double delayUs = 5;
	int n = serialDataAvail(fd);
	while(n < 1){
		delayMicroseconds(delayUs);
		n = serialDataAvail(fd);
	}
 	int foo = serialGetchar(fd);
 	while(foo == -1){
 		delayMicroseconds(delayUs);
 		foo = serialGetchar(fd);
 	}
    uint8_t RxSize = foo;//serialGetchar(fd);
    //uint8_t checksum = RxSize;
    //uint8_t *RxBuf = (uint8_t*)(malloc(RxSize * sizeof(uint8_t)));
    //std::vector<uint8_t> RxBuf(RxSize);
    std::vector<uint8_t> RxBuf;
    RxBuf.push_back(RxSize);
	//Test1
	n = serialDataAvail(fd);
	while(n < RxSize){
		delayMicroseconds(delayUs);
		n = serialDataAvail(fd);
	}
    for(int i = 0; i < RxSize; i++){
    	foo = serialGetchar(fd);
    	while(foo == -1){
    		delayMicroseconds(delayUs);
 			foo = serialGetchar(fd);
 		}
        RxBuf.push_back(foo);
        delayMicroseconds(delayUs);
        //delayMicroseconds(5);
    }*/
	//Test2
	/*
	for(int i = 0; i < RxSize; i++){
    	foo = serialGetchar(fd);
    	while(foo == -1){
 			foo = serialGetchar(fd);
 		}
        RxBuf.push_back(foo);
        delayMicroseconds(delayUs);
    }*/
	//Test3
	/*n = serialDataAvail(fd);
	while(n < RxSize){
		delayMicroseconds(delayUs);
		n = serialDataAvail(fd);
	}
	uint8_t *RxTab = (uint8_t*)(malloc(RxSize * sizeof(uint8_t)));
	if (read (fd, RxTab, RxSize) != RxSize)
		puts("READ ERROR in function DsPIC::read");
	for(int i = 0; i < RxSize; i++){
		RxBuf.push_back(RxTab[i]);
    }*/
    return RxBuf;
}
