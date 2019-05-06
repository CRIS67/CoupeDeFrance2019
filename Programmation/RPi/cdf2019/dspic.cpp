#include "dspic.hpp"

DsPIC::DsPIC(){
    fd = serialOpen ("/dev/serial0", BAUDRATE);
}
DsPIC::~DsPIC(){

}
/**
Initialize constant parameters of dspic : PID, odometry, maximum speed, maximum acceleration
*/
/*void DsPIC::initVarDspic(){
	//PID speed left
    setVarDouble64b(CODE_VAR_P_SPEED_L_LD,KP_SPEED_LEFT);
    setVarDouble64b(CODE_VAR_I_SPEED_L_LD,KI_SPEED_LEFT);
    setVarDouble64b(CODE_VAR_D_SPEED_L_LD,KD_SPEED_LEFT);
    //PID speed left
    setVarDouble64b(CODE_VAR_P_SPEED_R_LD,KP_SPEED_RIGHT);
    setVarDouble64b(CODE_VAR_I_SPEED_R_LD,KI_SPEED_RIGHT);
    setVarDouble64b(CODE_VAR_D_SPEED_R_LD,KD_SPEED_RIGHT);
    //PID distance
    setVarDouble64b(CODE_VAR_P_DISTANCE_LD,KP_DISTANCE);
    setVarDouble64b(CODE_VAR_I_DISTANCE_LD,KI_DISTANCE);
    setVarDouble64b(CODE_VAR_D_DISTANCE_LD,KD_DISTANCE);
    //PID angle
    setVarDouble64b(CODE_VAR_P_ANGLE_LD,KP_ANGLE);
    setVarDouble64b(CODE_VAR_I_ANGLE_LD,KI_ANGLE);
    setVarDouble64b(CODE_VAR_D_ANGLE_LD,KD_ANGLE);
	//Odometry
    setVarDouble64b(CODE_VAR_COEF_DISSYMETRY_LD,COEF_DISSYMETRY);
    setVarDouble64b(CODE_VAR_MM_PER_TICKS_LD,MM_PER_TICKS);
    setVarDouble64b(CODE_VAR_RAD_PER_TICKS_LD,RAD_PER_TICKS);
	//Linear speed & acceleration for trajectory generation
    setVarDouble64b(CODE_VAR_TRAJ_LIN_SPEED_LD,TRAJ_LIN_SPEED);
    setVarDouble64b(CODE_VAR_TRAJ_LIN_ACC_LD,TRAJ_LIN_ACC);
	//Angular speed & acceleration for trajectory generation
    setVarDouble64b(CODE_VAR_TRAJ_ROT_SPEED_LD,TRAJ_ROT_SPEED);
    setVarDouble64b(CODE_VAR_TRAJ_ROT_ACC_LD,TRAJ_ACC_SPEED);
}*/
/**
arg : 	<path> 	: path of config file ("config.txt" for example)

Initialize constant parameters of dspic from a config file : PID, odometry, maximum speed, maximum acceleration
*/
void DsPIC::loadVarDspicFromFile(std::string path){
	
	double kpR;
	double kiR;
	double kdR;
	
	double kpL;
	double kiL;
	double kdL;
	
	double kpD;
	double kiD;
	double kdD;
	
	double kpA;
	double kiA;
	double kdA;
	
	double coefDissymetry;
	double mmPerTick;
	double radPerTick;
	
	double linSpeed;
	double linAcc;
	double rotSpeed;
	double rotAcc;
	
	std::string line;
	std::ifstream myfile (path.c_str());
	if (myfile.is_open()){
		while (getline(myfile,line)){
			if(line.length() > 0){
				if(line.at(0) != '#'){
					//cout << line << '\n';
					std::string option,value;
					std::stringstream input_stringstream(line);
					getline(input_stringstream,option,'=');
					getline(input_stringstream,value,'=');
					//cout << option << '\n';
					//cout << value << '\n';
					
					//PID speed right
					if(option.compare("PidSpeedRightKp") == 0){
						std::stringstream ss(value);
						ss >> kpR;
					}
					else if(option.compare("PidSpeedRightKi") == 0){
						std::stringstream ss(value);
						ss >> kiR;
					}
					else if(option.compare("PidSpeedRightKd") == 0){
						std::stringstream ss(value);
						ss >> kdR;
					}
					//PID speed left
					else if(option.compare("PidSpeedLeftKp") == 0){
						std::stringstream ss(value);
						ss >> kpL;
					}
					else if(option.compare("PidSpeedLeftKi") == 0){
						std::stringstream ss(value);
						ss >> kiL;
					}
					else if(option.compare("PidSpeedLeftKd") == 0){
						std::stringstream ss(value);
						ss >> kdL;
					}
					//PID distance
					else if(option.compare("PidDistanceKp") == 0){
						std::stringstream ss(value);
						ss >> kpD;
					}
					else if(option.compare("PidDistanceKi") == 0){
						std::stringstream ss(value);
						ss >> kiD;
					}
					else if(option.compare("PidDistanceKd") == 0){
						std::stringstream ss(value);
						ss >> kdD;
					}
					//PID angle
					else if(option.compare("PidAngleKp") == 0){
						std::stringstream ss(value);
						ss >> kpA;
					}
					else if(option.compare("PidAngleKi") == 0){
						std::stringstream ss(value);
						ss >> kiA;
					}
					else if(option.compare("PidAngleKd") == 0){
						std::stringstream ss(value);
						ss >> kdA;
					}
					//Odometry
					else if(option.compare("coefDissymetry") == 0){
						std::stringstream ss(value);
						ss >> coefDissymetry;
					}
					else if(option.compare("mmPerTick") == 0){
						std::stringstream ss(value);
						ss >> mmPerTick;
					}
					else if(option.compare("radPerTick") == 0){
						std::stringstream ss(value);
						ss >> radPerTick;
					}
					//trajectory generation
					else if(option.compare("linSpeed") == 0){
						std::stringstream ss(value);
						ss >> linSpeed;
					}
					else if(option.compare("linAcc") == 0){
						std::stringstream ss(value);
						ss >> linAcc;
					}
					else if(option.compare("rotSpeed") == 0){
						std::stringstream ss(value);
						ss >> rotSpeed;
					}
					else if(option.compare("rotAcc") == 0){
						std::stringstream ss(value);
						ss >> rotAcc;
					}
				}
			}
		}
		myfile.close();
	}
	else{
		std::cout << "Unable to open file";
	}
	
	std::cout <<  kpR << std::endl;
	std::cout <<  kiR << std::endl;
	std::cout <<  kdR << std::endl;
	
	std::cout <<  kpL << std::endl;
	std::cout <<  kiL << std::endl;
	std::cout <<  kdL << std::endl;
	
	std::cout <<  kpD << std::endl;
	std::cout <<  kiD << std::endl;
	std::cout <<  kdD << std::endl;
	
	std::cout <<  kpA << std::endl;
	std::cout <<  kiA << std::endl;
	std::cout <<  kdA << std::endl;
	
	std::cout <<  coefDissymetry << std::endl;
	std::cout <<  mmPerTick << std::endl;
	std::cout <<  radPerTick << std::endl;
	
	std::cout <<  linSpeed << std::endl;
	std::cout <<  linAcc << std::endl;
	std::cout <<  rotSpeed << std::endl;
	std::cout <<  rotAcc << std::endl;
	
	//PID speed left
    setVarDouble64b(CODE_VAR_P_SPEED_L_LD,kpR);
    setVarDouble64b(CODE_VAR_I_SPEED_L_LD,kiR);
    setVarDouble64b(CODE_VAR_D_SPEED_L_LD,kdR);
    //PID speed left
    setVarDouble64b(CODE_VAR_P_SPEED_R_LD,kpL);
    setVarDouble64b(CODE_VAR_I_SPEED_R_LD,kiL);
    setVarDouble64b(CODE_VAR_D_SPEED_R_LD,kdL);
    //PID distance
    setVarDouble64b(CODE_VAR_P_DISTANCE_LD,kpD);
    setVarDouble64b(CODE_VAR_I_DISTANCE_LD,kiD);
    setVarDouble64b(CODE_VAR_D_DISTANCE_LD,kdD);
    //PID angle
    setVarDouble64b(CODE_VAR_P_ANGLE_LD,kpA);
    setVarDouble64b(CODE_VAR_I_ANGLE_LD,kiA);
    setVarDouble64b(CODE_VAR_D_ANGLE_LD,kdA);
	//Odometry
    setVarDouble64b(CODE_VAR_COEF_DISSYMETRY_LD,coefDissymetry);
    setVarDouble64b(CODE_VAR_MM_PER_TICKS_LD,mmPerTick);
    setVarDouble64b(CODE_VAR_RAD_PER_TICKS_LD,radPerTick);
	//Linear speed & acceleration for trajectory generation
    setVarDouble64b(CODE_VAR_TRAJ_LIN_SPEED_LD,linSpeed);
    setVarDouble64b(CODE_VAR_TRAJ_LIN_ACC_LD,linAcc);
	//Angular speed & acceleration for trajectory generation
    setVarDouble64b(CODE_VAR_TRAJ_ROT_SPEED_LD,rotSpeed);
    setVarDouble64b(CODE_VAR_TRAJ_ROT_ACC_LD,rotAcc);
}
/**
arg : 	<id> 	: [0;3] 		: 	id of servo (depends of number of available servomotors controllable by the dspic)
		<value> : [0;20000] µs 	: 	length of pulse in microseconds (µs) sent to control the servomotor

Move servo n° <id> to the position specified by <value>
*/
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
/**
arg : 	<id> 	: [0;1] 		: 	id of actuator motor [0-1] (depends of number of available motors controllable by the dspic)
		<value> : [-100;100] % 	: 	percentage (%) of duty cycle sent to motor

Change voltage applied to motor n° <id> to the value specified by <value> ( sign of value determine direction of rotation)
*/
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
/**
arg : 	<id> 	: [0;1] 		: 	id of actuator motor [0-1] (depends of number of available motors controllable by the dspic)
		<value> : [-Vbat;Vbat] % 	: 	voltage applied to the motor (max voltage is the battery voltage)

Change voltage applied to motor n° <id> to the value specified by <value> ( sign of value determine direction of rotation)
*/
void DsPIC::motorVoltage(uint8_t id, float value){
	uint8_t *ptr = (uint8_t*)&value;
    uint8_t buffer[RX_SIZE_MOTOR_VOLTAGE + 1];
    buffer[0] = RX_SIZE_MOTOR_VOLTAGE;
    buffer[1] = RX_CODE_MOTOR_VOLTAGE;
    buffer[2] = id;
    buffer[3] = ptr[0];
    buffer[4] = ptr[1];
    buffer[5] = ptr[2];
    buffer[6] = ptr[3];
    buffer[7] = 0;
    for(int i = 0; i < RX_SIZE_MOTOR_VOLTAGE; i++){
        buffer[7] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_MOTOR_VOLTAGE + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
/**
arg : 	<state> 	: [0;1] 		: 	0 -> out / 1 -> in

Extend or retract the linear arm
*/
void DsPIC::setMotLin(uint8_t state){
    uint8_t buffer[RX_SIZE_SET_MOT_LIN + 1];
    buffer[0] = RX_SIZE_SET_MOT_LIN;
    buffer[1] = RX_CODE_SET_MOT_LIN;
    buffer[2] = state;
    buffer[3] = 0;
    for(int i = 0; i < RX_SIZE_SET_MOT_LIN; i++){
        buffer[3] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_SET_MOT_LIN + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
/**
Enable propulsion motors
*/
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
/**
Disable propulsion motors
*/
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
/**
arg : 	<x> 		: [?] mm	: 	x position in mm. Must be in [0;2000] range if relative = 0 or in [-xRobot;2000-xRobot] range if relative = 1
		<y> 		: [?] mm	: 	y position in mm. Must be in [0;3000] range if relative = 0 or in [-yRobot;3000-yRobot] range if relative = 1
		<rev> 		: [0;1]  	: 	0 -> forward movement / -> backward movement
		<relative> 	: [0;1]  	: 	0 -> absolute position / -> relative position

Change voltage applied to motor n° <id> to the value specified by <value> ( sign of value determine direction of rotation)
*/
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
/**
arg : 	<t> 		: [?] deg	: 	angle in degrees
		<relative> 	: [0;1]  	: 	0 -> absolute angle / -> relative angle

Change voltage applied to motor n° <id> to the value specified by <value> ( sign of value determine direction of rotation)
*/
void DsPIC::turn(int16_t t, unsigned char relative){
    uint8_t option = 0;
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
void DsPIC::initPos(double x, double y, double t){
	//change position
    setVarDouble64b(CODE_VAR_X_LD,x);
    setVarDouble64b(CODE_VAR_Y_LD,y);
    setVarDouble64b(CODE_VAR_T_LD,t);
	//change setpoint
    setVarDouble64b(CODE_VAR_XC_LD,x);
    setVarDouble64b(CODE_VAR_YC_LD,y);
    setVarDouble64b(CODE_VAR_TC_LD,t);
	//change final setpoint
    setVarDouble64b(CODE_VAR_XF_LD,x);
    setVarDouble64b(CODE_VAR_YF_LD,y);
    setVarDouble64b(CODE_VAR_TF_LD,t);
}
/**
arg : 	<varCode> 	: [0;255] 				: 	code of desired variable (see #define)
		<var> 		: decimal real value  	: 	value of the variable

Set value of a 64bits decimal variable of dspic ("long double" on the dspic / "double" on raspbian).
*/
void DsPIC::setVarDouble64b(uint8_t varCode, double Var){
    double *ptrVar = &Var;
    uint8_t *ptr = (uint8_t*)ptrVar;
    uint8_t buffer[RX_SIZE_SET_64b + 1];
    buffer[0] = RX_SIZE_SET_64b;
    buffer[1] = RX_CODE_SET;
    buffer[2] = varCode;
    buffer[3] = VAR_LD_64b;
    buffer[4] = ptr[0];
    buffer[5] = ptr[1];
    buffer[6] = ptr[2];
    buffer[7] = ptr[3];
    buffer[8] = ptr[4];
    buffer[9] = ptr[5];
    buffer[10] = ptr[6];
    buffer[11] = ptr[7];
    buffer[12] = 0;
    for(int i = 0; i < RX_SIZE_SET_64b; i++){
        buffer[12] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_SET_64b + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
/**
arg : 	<varCode> 	: [0;255] 			: 	code of desired variable (see #define)
		<var> 		: [0;4294967295]  	: 	value of the variable

Set value of an unsigned or signed integer 32bits variable of dspic (uint32_t / int32_t). Signed integer can be passed as unsigned integer and will be casted to integer when received by the dspic
*/
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
/**
arg : 	<varCode> 	: [0;255] 	: 	code of desired variable (see #define)
		<var> 		: [0;255]  	: 	value of the variable

Set value of an unsigned or signed integer 8bits variable of dspic (uint8_t / int8_t). Signed integer can be passed as unsigned integer and will be casted to integer when received by the dspic
*/
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
/**
arg : 	<varCode> 		: [0;255] 	: 	code of desired variable (see #define)

Ask for variable n° <varCode>. Respone will be processed in the UART reception thread
*/
void DsPIC::getVar(uint8_t varCode){
    uint8_t buffer[RX_SIZE_GET + 1];
    buffer[0] = RX_SIZE_GET;
    buffer[1] = RX_CODE_GET;
    buffer[2] = varCode;
    buffer[3] = 0;
    for(int i = 0; i < RX_SIZE_GET; i++){
        buffer[3] += buffer[i]; //checksum
    }
    for(int i = 0; i < RX_SIZE_GET + 1; i++){
        serialPutchar (fd, buffer[i]);
    }
}
/**
Ask for all PID variables. Respone will be processed in the UART reception thread
*/
void DsPIC::loadPID(){
    getVar(CODE_VAR_ALLPID);
}
/**
Clear content of UART reception buffer
*/
std::string DsPIC::async_read(){
    std::string s("");
    while (serialDataAvail(fd)){
      s += serialGetchar(fd);
    }
    return s;
}
/**
Wait for a complete packet ( size + header + content + checksum ) and return it as a vector of bytes.

return value : vector containing bytes of a complete packet
*/
 std::vector<uint8_t> DsPIC::readMsg(){
	int receiveByte = serialGetchar(fd);
 	while(receiveByte == -1){	//no reception during 10 sec
 		receiveByte = serialGetchar(fd);
 	}
 	uint8_t RxSize = receiveByte;	//first byte is the size of the packet
    std::vector<uint8_t> RxBuf;
    RxBuf.push_back(RxSize);

	for(int i = 0; i < RxSize; i++){	//receive full packet
    	receiveByte = serialGetchar(fd);
    	while(receiveByte == -1){
 			receiveByte = serialGetchar(fd);
 		}
        RxBuf.push_back(receiveByte);
    }
    return RxBuf;
}
