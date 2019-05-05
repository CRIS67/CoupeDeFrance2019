#include "web.hpp"

Web::Web(DsPIC *ds){
	dspic = ds;
    s = "hello world !";
    struct sockaddr_in server;

    //Create socket
    socket_listen = socket(AF_INET , SOCK_STREAM , 0);

	const int optVal = 1;
	const socklen_t optLen = sizeof(optVal);
	setsockopt(socket_listen, SOL_SOCKET, SO_REUSEADDR, (void*) &optVal, optLen);

    if (socket_listen == -1)
    {
        printf("Could not create socket");
    }

    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( 8888 );

    //Bind
    if( bind(socket_listen,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
        puts("bind failed");
		perror("bind");
		exit(EXIT_FAILURE);
    }
    puts("bind done");

    //Listen
    listen(socket_listen , 3);

    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    socket_client = -1;
}
Web::~Web(){
	close(socket_listen);
	close(socket_client);
    //dtor
}
bool Web::acceptClient(){
    int c = sizeof(struct sockaddr_in);
    struct sockaddr_in client;
    socket_client = accept(socket_listen, (struct sockaddr *)&client, (socklen_t*)&c);
    if (socket_client < 0)
    {
        perror("accept failed");
        return false;
    }
    //puts("Connection accepted");
    return true;
}
void Web::closeClient(){
	close(socket_client);
}
bool Web::sendMsg(std::string message){
    char const *cMessage = message.c_str();
    short msgSize = strlen(cMessage);
    char L = (char)(msgSize & 0xFF);
    char H = (char)(msgSize >> 8);
    send(socket_client, &H, 1, 0);
    send(socket_client, &L, 1, 0);
    if(send(socket_client , cMessage , strlen(cMessage) , 0) == -1)
        return false;
    return true;
}
std::string Web::receiveMsg(){
	//puts("start receive");
	char RxSizeBuf[2];
    recv(socket_client, RxSizeBuf, 2, 0);
	int RxSize = (RxSizeBuf[0] << 8) + RxSizeBuf[1];
	char *RxBuf = (char*)(malloc(RxSize + 1 * sizeof(char)));
	recv(socket_client, RxBuf, RxSize, 0);
	RxBuf[RxSize] = '\0';
	//printf("n = %d\nsize = %d\n",n,RxSize);
	//puts(RxBuf);
	std::string str(RxBuf);
	free(RxBuf);
	//puts("end receive");
	return str;
}
bool Web::startThread(){
   int rc;
    std::cout << "Web::ctor() : creating thread, " << std::endl;
    rc = pthread_create(&threads, NULL, thread_HandleConnnection, (void*)this);

    if (rc) {
    std::cout << "Error:unable to create thread," << rc << std::endl;
    return false;
    }
    return true;
}
void* thread_HandleConnnection(void *threadid){
   Web *w = (Web*)threadid;
   std::cout << "Hello World!" << std::endl;
   double i = 0;
   char msg_arr[100];
   while(1){
	    i += 0.05;
        w->acceptClient();
		//printf("Connection accepted : ");
		std::string str = w->receiveMsg();
		const char *cmsg = str.c_str();
		strcpy(msg_arr,cmsg);

		char *savePtrMsg, *savePtr;
		char *msg = strtok_r(msg_arr,"&",&savePtrMsg);
		char *sub, *cmd, *opt, *val;

		std::string sendStr = "invalid cmd";
		std::string errorStr = "";
		bool error = false;

		bool rX = false;
		bool rY = false;

		int x,y;

		while(msg != NULL){	//for each message
			sub = strtok_r(msg,"=",&savePtr);
			val = strtok_r(NULL,"=",&savePtr);
			cmd = strtok_r(sub,":",&savePtr);
			opt = strtok_r(NULL,"=",&savePtr);

			if(cmd == NULL){
				errorStr += "Error : cmd token = NULL\n";
				error = true;
				break;
			}

			if(!strcmp(cmd,"update")){
				//puts("update");
				//sendStr = simulateResponse(i);
				sendStr = realResponse(w);
				break;	//if update => no other cmd executed
			}
			else if(!strcmp(cmd,"stop")){
				if(val == NULL){
					errorStr += "Error : stop value token = NULL\n";
					error = true;
					break;
				}
				if(atoi(val) == 1){
					w->dspic->stop();
				}
				
			}
			else if(!strcmp(cmd,"x")){
				if(val == NULL){
					errorStr += "Error : x value token = NULL\n";
					error = true;
					break;
				}
				x = atoi(val);
				if(x >= 0 && x <= 3000){
					sendStr = "OK";
					printf("variable set : xDest = %d\n",x);
					rX = true;
				}
				else{
					errorStr += "Error x range\n";
					error = true;
					puts("Error x range");
					rX = false;
				}
			}
			else if(!strcmp(cmd,"y")){
				if(val == NULL){
					errorStr += "Error : y value token = NULL\n";
					error = true;
					break;
				}
				y = atoi(val);
				if(y >= 0 && y <= 2000){
					sendStr = "OK";
					printf("variable set : yDest = %d\n",y);
					rY = true;
				}
				else{
					errorStr += "Error y range\n";
					error = true;
					puts("Error y range");
					rY = false;
				}
			}
			else if(!strcmp(cmd,"go")){
				if(rX && rY){
					if(opt != NULL && !strcmp(opt,"rel")){
						printf("command executed : go relative %d %d \n",x,y);
						w->dspic->go(x,y,0,1);
					}
					else{
						printf("command executed : go absolute %d %d \n",x,y);
						w->dspic->go(x,y,0,0);
					}
					sendStr = "OK";
				}
				else{
					errorStr += "Variables x & y need to be set before executing function go\n";
					error = true;
					puts("Variables x & y need to be set before executing function go");
				}
			}
			else if(!strcmp(cmd,"gb")){
				if(rX && rY){
					if(opt != NULL && !strcmp(opt,"rel")){
						printf("command executed : go back relative %d %d \n",x,y);
						w->dspic->go(x,y,1,1);
					}
					else{
						printf("command executed : go back absolute %d %d \n",x,y);
						w->dspic->go(x,y,1,0);
					}
					sendStr = "OK";

				}
				else{
					errorStr += "Variables x & y need to be set before executing function go\n";
					error = true;
					puts("Variables x & y need to be set before executing function go");
				}
			}
			else if(!strcmp(cmd,"turn")){
				if(val == NULL){
					errorStr += "Error : turn value token = NULL\n";
					error = true;
					puts("Error : turn value token ");
					break;
				}
				int angle = atoi(val);
				if(opt != NULL && !strcmp(opt,"rel")){
					printf("command executed : turn relative %d degree\n",angle);
					w->dspic->turn(angle,0,1);
				}
				else{
					printf("command executed : turn absolute %d degree\n",angle);
					w->dspic->turn(angle,0,0);
				}
				sendStr = "OK";
			}
			else if(!strcmp(cmd,"s")){
				if(opt == NULL){
					errorStr += "Error : servo id token = NULL\n";
					error = true;
					puts("Error : servo id token ");
					break;
				}
				if(val == NULL){
					errorStr += "Error : servo value token = NULL\n";
					error = true;
					puts("Error : servo value token ");
					break;
				}
				int id = atoi(opt);
				if(id >= 0 && id <= 6){
					int n = atoi(val);
					if(n >= 0 && n <= 20000){
						sendStr = "OK : command executed : servo " + std::to_string(id) + " " + std::to_string(n);
						printf("command executed : servo %d %d\n",id,n);
						w->dspic->servo(id,n);
					}
					else{
						errorStr += "Error : servo range second token\n";
						error = true;
						puts("Error : servo range second token ");
					}
				}
				else{
					errorStr += "Error : invalid servo id\n";
					error = true;
					puts("error invalid servo id");
				}
			}
			else if(!strcmp(cmd,"sp")){
				if(opt == NULL){
					errorStr += "Error : servo preset id token = NULL\n";
					error = true;
					puts("Error : servo preset id token ");
					break;
				}
				if(val == NULL){
					errorStr += "Error : servo preset value token = NULL\n";
					error = true;
					puts("Error : servo preset value token ");
					break;
				}
				int id = atoi(opt);
				if(id >= 0 && id <= 6){
					int n = atoi(val);
					if(n >= 0 && n <= 2){
						sendStr = "OK";
						printf("command executed : servo %d preset %d\n",id,n);
					}
					else{
						errorStr += "Error : servo preset value second token\n";
						error = true;
						puts("Error : servo preset value second token");
					}
				}
				else{
					errorStr += "Error : invalid servo preset id\n";
					error = true;
					puts("Error : invalid servo preset id");
				}
			}
			else if(!strcmp(cmd,"m")){
				if(opt == NULL){
					errorStr += "Error : motor id token = NULL\n";
					error = true;
					puts("Error : motor id token = NULL\n");
					break;
				}
				if(val == NULL){
					errorStr += "Error : motor value token = NULL\n";
					error = true;
					puts("Error : motor value token = NULL\n");
					break;
				}
				int id = atoi(opt);
				if(id >= 0 && id <= 4){
					int n = atoi(val);
					if(n >= 0 && n <= 100){
						sendStr = "OK";
						printf("command executed : motor %d %d\n",id,n);
						w->dspic->motor(id,n);
					}
					else{
						errorStr += "Error : motor value second token\n";
						error = true;
						puts("Error : motor value second token");
					}
				}
				else{
					errorStr += "Error : invalid motor id\n";
					error = true;
					puts("Error : invalid motor id");
				}
			}
			else if(!strcmp(cmd,"mp")){

				if(opt == NULL){
					errorStr += "Error : motor preset id token = NULL\n";
					error = true;
					puts("Error : motor preset id token = NULL\n");
					break;
				}
				if(val == NULL){
					errorStr += "Error : motor preset value token = NULL\n";
					error = true;
					puts("Error : motor preset value token = NULL\n");
					break;
				}
				int id = atoi(opt);
				if(id >= 0 && id <= 4){
					int n = atoi(val);
					if(n >= 0 && n <= 2){
						sendStr = "OK";
						printf("command executed : motor %d preset %d\n",id,n);
					}
					else{
						errorStr += "Error : motor preset value second token\n";
						error = true;
						puts("Error : motor preset value second token");
					}
				}
				else{
					errorStr += "Error : invalid motor preset id\n";
					error = true;
					puts("Error : invalid motor preset id");
				}
			}
			else if(!strcmp(cmd,"p1")){
				if(val == NULL){
					errorStr += "Error : P1 value token = NULL\n";
					error = true;
					puts("Error : motor P1 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_P_SPEED_L,value);
				printf("P1 (Speed Left) set to %u\n", value);
			}
			else if(!strcmp(cmd,"i1")){
				if(val == NULL){
					errorStr += "Error : I1 value token = NULL\n";
					error = true;
					puts("Error : motor I1 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_I_SPEED_L,value);
				printf("I1 (Speed Left) set to %u\n", value);
			}
			else if(!strcmp(cmd,"d1")){
				if(val == NULL){
					errorStr += "Error : D1 value token = NULL\n";
					error = true;
					puts("Error : motor D1 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_D_SPEED_L,value);
				printf("D1 (Speed Left) set to %u\n", value);
			}
			else if(!strcmp(cmd,"p2")){
				if(val == NULL){
					errorStr += "Error : P2 value token = NULL\n";
					error = true;
					puts("Error : motor P2 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_P_SPEED_R,value);
				printf("\nP2 (Speed Right) set to %u\n", value);
			}
			else if(!strcmp(cmd,"i2")){
				if(val == NULL){
					errorStr += "Error : I2 value token = NULL\n";
					error = true;
					puts("Error : motor I2 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_I_SPEED_R,value);
				printf("I2 (Speed Right) set to %u\n", value);
			}
			else if(!strcmp(cmd,"d2")){
				if(val == NULL){
					errorStr += "Error : D2 value token = NULL\n";
					error = true;
					puts("Error : motor D2 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_D_SPEED_R,value);
				printf("D2 (Speed Right) set to %u\n", value);
			}
			else if(!strcmp(cmd,"p3")){
				if(val == NULL){
					errorStr += "Error : P3 value token = NULL\n";
					error = true;
					puts("Error : motor P3 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_P_DISTANCE,value);
				printf("\nP3 (Distance) set to %u\n", value);
			}
			else if(!strcmp(cmd,"i3")){
				if(val == NULL){
					errorStr += "Error : I3 value token = NULL\n";
					error = true;
					puts("Error : motor I3 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_I_DISTANCE,value);
				printf("I3 (Distance) set to %u\n", value);
			}
			else if(!strcmp(cmd,"d3")){
				if(val == NULL){
					errorStr += "Error : D3 value token = NULL\n";
					error = true;
					puts("Error : motor D3 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_D_DISTANCE,value);
				printf("D3 (Distance) set to %u\n", value);
			}
			else if(!strcmp(cmd,"p4")){
				if(val == NULL){
					errorStr += "Error : P4 value token = NULL\n";
					error = true;
					puts("Error : motor P4 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_P_ANGLE,value);
				printf("\nP4 (Angle) set to %u\n", value);
			}
			else if(!strcmp(cmd,"i4")){
				if(val == NULL){
					errorStr += "Error : I4 value token = NULL\n";
					error = true;
					puts("Error : motor I4 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_I_ANGLE,value);
				printf("I4 (Angle) set to %u\n", value);
			}
			else if(!strcmp(cmd,"d4")){
				if(val == NULL){
					errorStr += "Error : D4 value token = NULL\n";
					error = true;
					puts("Error : motor D4 token = NULL\n");
					break;
				}
				int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_D_ANGLE,value);
				printf("D4 (Angle) set to %u\n", value);
			}
			else if(!strcmp(cmd,"loadPID")){
				w->dspic->loadPID();
				w->waitingResponsePID = true;
				w->dspic->isPIDUpdated = false;
			}
			else if(!strcmp(cmd,"odo1")){
				if(val == NULL){
					errorStr += "Error : COEF_DISSYMETRY value token = NULL\n";
					error = true;
					puts("Error : COEF_DISSYMETRY token = NULL\n");
					break;
				}
				/*int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_COEF_DISSYMETRY,value);
				printf("COEF_DISSYMETRY set to %u\n", value);*/
				double doubleVal = atof(val);
				w->dspic->setVarDouble64b(CODE_VAR_COEF_DISSYMETRY_LD,doubleVal);
				printf("COEF_DISSYMETRY set to %f\n", doubleVal);
			}
			else if(!strcmp(cmd,"odo2")){
				if(val == NULL){
					errorStr += "Error : MM_PER_TICKS value token = NULL\n";
					error = true;
					puts("Error : MM_PER_TICKS token = NULL\n");
					break;
				}
				/*int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_MM_PER_TICKS,value);
				printf("MM_PER_TICKS set to %u\n", value);*/
				double doubleVal = atof(val);
				w->dspic->setVarDouble64b(CODE_VAR_MM_PER_TICKS_LD,doubleVal);
				printf("MM_PER_TICKS set to %f\n", doubleVal);
			}
			else if(!strcmp(cmd,"odo3")){
				if(val == NULL){
					errorStr += "Error : RAD_PER_TICKS value token = NULL\n";
					error = true;
					puts("Error : RAD_PER_TICKS token = NULL\n");
					break;
				}
				/*int32_t longVal = atol(val);
				uint32_t value = (uint32_t)longVal;
				w->dspic->setVar32(CODE_VAR_DISTANCE_BETWEEN_ENCODER_WHEELS,value);
				printf("DISTANCE_BETWEEN_ENCODER_WHEELS set to %u\n", value);*/
				double doubleVal = atof(val);
				w->dspic->setVarDouble64b(CODE_VAR_RAD_PER_TICKS_LD,doubleVal);
				printf("RAD_PER_TICKS set to %f\n", doubleVal);
			}
			else{
				errorStr += "Received command is not valid : " + str + "\n";
				error = true;
				//puts("Received command is not valid");
				printf("Received command is not valid : %s\n",cmsg);
			}


			msg = strtok_r(NULL,"&",&savePtrMsg);
		}

		//char *token = strtok(msg,"=");

		//token = strtok(msg,"-");
		if(error)
			w->sendMsg(errorStr);
		else
			w->sendMsg(sendStr);
		w->closeClient();
   }
   pthread_exit(NULL);
}
std::string simulateResponse(double i){
	std::ostringstream myString;
	myString << "x=";
	myString << 1500 + 100*sin(i);
	myString << "&y=";
	myString << 1000 + 100*cos(i);
	myString << "&t=";
	myString << i*10;
	myString << "&c1=";
	myString << i;
	myString << ";";
	myString << cos(i);
	myString << "&c2=";
	myString << i;
	myString << ";";
	myString << sin(i);
	std::string str = myString.str();
	return str;
}
std::string realResponse(Web *w){
	DsPIC *dspic = w->dspic;
	std::ostringstream myString;
	myString << "x=";
	myString << dspic->x;
	myString << "&y=";
	myString << dspic->y;
	myString << "&t=";
	myString << dspic->t;
	
	myString << "&b=";
	myString << dspic->bat;

	myString << "&r1=";
	myString << dspic->rupt.ass0;
	myString << "&r2=";
	myString << dspic->rupt.ass1;
	myString << "&r3=";
	myString << dspic->rupt.ass2;
	myString << "&r4=";
	myString << dspic->rupt.ass3;
	myString << "&r5=";
	myString << dspic->rupt.act0;
	myString << "&r6=";
	myString << dspic->rupt.act1;
	myString << "&r7=";
	myString << dspic->rupt.act2;
	myString << "&r8=";
	myString << dspic->rupt.act3;
	myString << "&r9=";
	myString << dspic->rupt.act4;
	myString << "&r10=";
	myString << dspic->rupt.act5;

	for(int i = 0; i < 6; i++){
        myString << "&d" << i + 1 << "=";
	myString << dspic->US[i];
	}
	while(dspic->logs.size() > 0){
        myString << "&l=";
        std::string qs = dspic->logs.front();
        dspic->logs.pop();
        myString << qs;
        //std::cout << "log sent to web -> " << qs << std::endl;
        //myString << qs;
	}
	while(dspic->plots.size() > 0){
        point p = dspic->plots.front();
        dspic->plots.pop();
        myString << "&c" << (int)p.id << "=" << p.x << ";" << p.y;
        //std::cout << "plot : " << (int)p.id << "=" << p.x << ";" << p.y << std::endl;
        //myString << qs;
	}
	if(w->waitingResponsePID && w->dspic->isPIDUpdated){
        w->waitingResponsePID = false;
		myString << "&p1=";
		myString << dspic->pidSpeedLeft.Kp;
		myString << "&i1=";
		myString << dspic->pidSpeedLeft.Ki;
		myString << "&a1=";
		myString << dspic->pidSpeedLeft.Kd;

		myString << "&p2=";
		myString << dspic->pidSpeedRight.Kp;
		myString << "&i2=";
		myString << dspic->pidSpeedRight.Ki;
		myString << "&a2=";
		myString << dspic->pidSpeedRight.Kd;

		myString << "&p3=";
		myString << dspic->pidDistance.Kp;
		myString << "&i3=";
		myString << dspic->pidDistance.Ki;
		myString << "&a3=";
		myString << dspic->pidDistance.Kd;

		myString << "&p4=";
		myString << dspic->pidAngle.Kp;
		myString << "&i4=";
		myString << dspic->pidAngle.Ki;
		myString << "&a4=";
		myString << dspic->pidAngle.Kd;
	}
	/*myString << "&c1=";
	myString << i;
	myString << ";";
	myString << cos(i);
	myString << "&c2=";
	myString << i;
	myString << ";";
	myString << sin(i);*/
	std::string str = myString.str();
	return str;
}
