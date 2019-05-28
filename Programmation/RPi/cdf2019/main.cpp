#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <vector>
#include <queue>
#include "web.hpp"
#include "dspic.hpp"
#include "actuators.hpp"
#include "SPI.hpp"
#include "lidar.hpp"
#include "actuators.hpp"
#include "hmi.hpp"

#include <wiringPi.h>
#include <wiringPiSPI.h>

// DStarIncludes 
#include <utility>
#include <algorithm>
#include <cmath>
#include <bits/stdc++.h>
#include <limits>
#include <map>
#include "mapGeneration.hpp"
#include "dStarLite.hpp"
#include "trajectoryHandle.hpp"


#include <fstream>
#include <sstream>

#define DEBUG_ENABLE_PRINT			0

#if(DEBUG_ENABLE_PRINT)
#define DEBUG_PRINT(x) 				std::cout << "DEBUG_PRINT>> " << x << std::endl;
#else
#define DEBUG_PRINT(x)				
#endif

#define MAP_MM_PER_ARRAY_ELEMENT 	20


#define MM_ROWS						2000
#define MM_COLUMNS					3000

#define MM_START_X					1000
#define MM_START_Y					2500

#define MM_SIZE_ENNEMY 				200
#define MM_SIZE_ENNEMY_MARGIN		220

#define SIZE_ENNEMY					MM_SIZE_ENNEMY / MAP_MM_PER_ARRAY_ELEMENT	//size of ennemy /2 4 element in array map -> (here 40cm)WARNING depends of map resolution
#define SIZE_ENNEMY_MARGIN			MM_SIZE_ENNEMY_MARGIN / MAP_MM_PER_ARRAY_ELEMENT

#define ARRAY_START_X				MM_START_X / MAP_MM_PER_ARRAY_ELEMENT
#define ARRAY_START_Y				MM_START_Y / MAP_MM_PER_ARRAY_ELEMENT
//DStarGlobal 
int mapRows {MM_ROWS / MAP_MM_PER_ARRAY_ELEMENT};  
int mapColumns {MM_COLUMNS / MAP_MM_PER_ARRAY_ELEMENT};  
int km {0}; // variable for the D*

std::vector<std::vector<int>> mapVector; // the robot's map 

bool obstacleDetection {false}; 
bool pointReached {false}; 
 
Node startNode = {infinity,infinity,0,std::pair<int,int>(ARRAY_START_X,ARRAY_START_Y)};
Node goalNode  = {infinity,0,0,std::pair<int,int>(9,9), false};

priorityList uList; // priority List
mappedNodes knownNodes; // node the robot can see


const int enemyWidth = 500; // The distance the enemy takes place on the map, it is represented as a square 

std::vector<Node> simplifiedPath; 
std::vector<Node> completePath; 

// Strategy include 
#include "LK.h" 

#define PIN_JUMPER		27

#define HALF_LENGTH		124
#define HALF_WIDTH		155

#define PI 				3.14159

#define FORWARD 		0
#define BACKWARD 		1

void debugAct();
void debugTestAllDelay();
void debugTestAllInstant();
void debugBN();
void debugGoldenium();
int main()
{
	//debugAct();
	//exit(0);
	std::cout << std::endl << "     Coupe de France 2019 by CRIS" << std::endl << std::endl;
    wiringPiSetup();
	
    pinMode(PIN_JUMPER,INPUT);

    std::cout << "Initialisation..." << std::endl;

    DsPIC dspic;
    dspic.reset();
    delay(250); //wait for reset
    dspic.async_read(); //flush rx buffer

    Web web(&dspic);

    SPI spi(SPI_CHANNEL,SPI_SPEED); //initialise SPI
	Actuators actFront(&spi,SPI_ID_ACT_FRONT), actBack(&spi,SPI_ID_ACT_BACK);
	actFront.ResetAtmega();
	actBack.ResetAtmega();
    Lidar lidar(&spi, SPI_ID_LIDAR, &web);
    lidar.stop();
    HMI hmi(&spi,SPI_ID_HMI);

    puts("Hello human ! I, your fervent robot, am initialised. Press <ENTER> to continue.");
	getchar();

	lidar.start();
    dspic.startThreadReception();
    web.startThread();
    lidar.startThreadDetection();

    dspic.setVar8(CODE_VAR_VERBOSE,1);
    puts("verbose set to 1");
    dspic.getVar(CODE_VAR_BAT);
    dspic.loadVarDspicFromFile("config.txt");
    //dspic.initPos(1000,1500,3.14159);
    //dspic.initPos(1200,2550,-3.14159/2);
    //dspic.initPos(startNode.coord.first*MAP_MM_PER_ARRAY_ELEMENT,startNode.coord.second*MAP_MM_PER_ARRAY_ELEMENT, -3.14159/2);
    int xInit = 1200-HALF_WIDTH;
    int yInit = 2550+HALF_LENGTH;
    double tInit = -PI/2;
    //double tInit = 0;
    dspic.initPos(xInit,yInit,tInit);
    //dspic.initPos(1000,1500,-3.14159/2);
    //dspic.initPos(0,0,0);
    //dspic.initPos(1000,3000,-3.14159/2);
	
    std::cout << "Press enter to dspic.start() " << std::endl; 
    //getchar();

    //dspic.reset();
    getchar();
    dspic.start();
	/*while(1){
		std::ofstream outfile;

		outfile.open("pos.txt", std::ios_base::app);
		outfile << dspic.getX() << " " << dspic.getY() << " " << dspic.getT() << std::endl;
		delay(10);
	}*/
    std::cout << "Press enter to test the brake" << std::endl; 
    getchar(); 
	dspic.setVarDouble64b(CODE_VAR_TRAJ_LIN_SPEED_LD,500);
	dspic.go(1045,1500,0,0);
	std::cout << "Press enter to brake" << std::endl;
	getchar(); 
	dspic.brake();	
	std::cout << "Press enter to stop" << std::endl;
	getchar();
	dspic.stop();	
	std::cout << "Press enter to start" << std::endl;
	getchar();
	dspic.start();
	dspic.setVarDouble64b(CODE_VAR_TRAJ_LIN_SPEED_LD,200);
    getchar(); 
    
    std::vector<int> vSetPointX;
    std::vector<int> vSetPointY;
    std::vector<int> vSetPointDirection;
    int xSetpoint = xInit;
    int ySetpoint = yInit;
    vSetPointX.push_back(xInit);
    vSetPointY.push_back(2429);
    vSetPointDirection.push_back(FORWARD);
    vSetPointX.push_back(1400);
    vSetPointY.push_back(2420);
    vSetPointDirection.push_back(BACKWARD);
    vSetPointX.push_back(xInit);
    vSetPointY.push_back(2429);
    vSetPointDirection.push_back(FORWARD);
    vSetPointX.push_back(xInit);
    vSetPointY.push_back(2770);
    vSetPointDirection.push_back(BACKWARD);
    bool started = true;
    int nDetection = 0;

    // Map Generation 
    generateMap(mapVector,mapRows,mapColumns); // generates empty map 
    createRectangle(0,0,300/MAP_MM_PER_ARRAY_ELEMENT,mapColumns, mapVector); // creates a 400x2000 obstacle rectangle  at (1600,0) 
    createRectangle(1700/MAP_MM_PER_ARRAY_ELEMENT,0,250 / MAP_MM_PER_ARRAY_ELEMENT,mapColumns, mapVector); // creates a 400x2000 obstacle rectangle  at (1600,0) 
    //createRectangle(90,140,20,20, mapVector); // creates a 400x2000 obstacle rectangle  at (1600,0) 
    std::vector<std::vector<int>> debugMap(mapVector); 

    while(1){
    	int nbPath = 0;
    	while(digitalRead(PIN_JUMPER));
    	while(!digitalRead(PIN_JUMPER));
    	for(unsigned int i = 0; i < vSetPointX.size();i++){
    		std::cout << "i = " << i;
    		switch(i){
    			case 0:
    				actBack.SetPump(1,1);
    				actBack.MoveServo(1,SERVO_VALUE_LOW);
    				delay(1000);
    				actBack.MoveServo(1,SERVO_VALUE_HIGH);
    				break;
    			case 2:
    				delay(1000);
    				actFront.SetPump(0,1);
    				actFront.SetPump(1,1);
    				actFront.SetPump(2,1);
    				actFront.MoveServo(0,900);
    				actFront.MoveServo(1,900);
    				actFront.MoveServo(2,900);
    				delay(500);
    				actFront.MoveServo(0,SERVO_VALUE_HIGH);
    				actFront.MoveServo(1,SERVO_VALUE_HIGH);
    				actFront.MoveServo(2,SERVO_VALUE_HIGH);
    				break;
    		}
    		int xSetpoint = vSetPointX.at(i);
    		std::cout << " / x";
			int ySetpoint = vSetPointY.at(i);
			std::cout << " / y";
			int dir = vSetPointDirection.at(i);
			std::cout << " / t" <<std::endl;;
			std::cout << "send go " << xSetpoint << " ; " << ySetpoint << std::endl;
			dspic.go(xSetpoint, ySetpoint,dir,0); // we move the robot to the next point

			lidar.setFillBuffer(true);	//keep track of the points detected by lidar in a member buffer
			while((dspic.getX() - xSetpoint)*(dspic.getX() - xSetpoint) + (dspic.getY() - ySetpoint)*(dspic.getY() - ySetpoint) > 400){
				//get lidar points
				std::queue<pointFloat2d> points = lidar.getAndClearDetectedPoints();	//get detected points and clear internal buffer
				std::queue<pointFloat2d> savePoints = points;
				unsigned int nbPoints = points.size();
				//std::cout << "nbPoints lidar = " << nbPoints << std::endl;
				//add on binary map
				std::vector<std::vector<int>> newMap(mapVector);
				for(unsigned int i = 0; i < nbPoints;i++){
					pointFloat2d p = points.front();
					points.pop();
					int x = (int)round(p.x / MAP_MM_PER_ARRAY_ELEMENT);
					int y = (int)round(p.y / MAP_MM_PER_ARRAY_ELEMENT);
					//std::cout << "(" << x << " ; " << y << ") ";
					for(int j = x - SIZE_ENNEMY; j < x + SIZE_ENNEMY;j++){
						for(int k = y - SIZE_ENNEMY; k < y + SIZE_ENNEMY;k++){
							if(j >= 0 && j < mapRows && k >= 0 && k < mapColumns){
								newMap.at(j).at(k) = 1;
								//std::cout << "(" << j << " ; " << k << ") ";
							}
						}
					}
					//std::cout << std::endl;
				}
				//detect Collision
				Node nRobot;
				nRobot.coord.first = (int)(dspic.getX()/MAP_MM_PER_ARRAY_ELEMENT);
				nRobot.coord.second = (int)(dspic.getY()/MAP_MM_PER_ARRAY_ELEMENT);
				//DEBUG_PRINT("detect collision begin");
				//bool obstacleDetection = detectCollision(newMap,simplifiedPath, nRobot, startNode);	//ATTENTION à changer plus tard pour ne pas prendre en compte la partie du trajet déjà fait
				int xSPArray = xSetpoint / MAP_MM_PER_ARRAY_ELEMENT;
				int ySPArray = ySetpoint / MAP_MM_PER_ARRAY_ELEMENT;
				bool obstacleDetection = detectCollisionLine(nRobot.coord.first, nRobot.coord.second,xSPArray, ySPArray,newMap);
				//DEBUG_PRINT("detect collision ended");
				//bool obstacleDetection = detectCollisionLine(100,50,100,250,newMap);	//ATTENTION à changer plus tard pour ne pas prendre en compte la partie du trajet déjà fait
				if(obstacleDetection){
					puts("detection");
					nDetection = 0;
					if(started){
						started = false;
						dspic.stop();
					}
				}
				else{
					//puts("no detection");
					if(nDetection > 10){
						if(!started){
							dspic.start();
							dspic.go(xSetpoint, ySetpoint,dir,0);
							//delay(10);
							started = true;
						}
					}
					else
						nDetection++;
				}
				
				DEBUG_PRINT("printing MAP");
				std::ofstream file;
				std::ostringstream ss;
				//ss << "logMaps/out_map" << nbStrat << "_" << nbPath++ << ".txt";
				ss << "logMaps/out_map" << nbPath++ << ".txt";
				file.open(ss.str());
				/*for(unsigned int i = 0; i < simplifiedPath.size();i++){
					newMap.at(simplifiedPath.at(i).coord.first).at(simplifiedPath.at(i).coord.second) = 4;
				}*/
				newMap.at(nRobot.coord.first).at(nRobot.coord.second) = 3;
				for(int i = 0; i < mapRows;i++){
					for(int j = 0; j < mapColumns; j++){
						file << newMap.at(i).at(j) << " ";
					}
					file << std::endl;
				}
				file.close();	
				DEBUG_PRINT("printing MAP ended");
				//printMap(mapRows,mapColumns,newMap);
				delay(50);
			}
			puts("debug2");
			lidar.setFillBuffer(false);	//stop keeping lidar points to avoid running out of memory
			puts("debug3");
		}	
		actFront.MoveServo(0,SERVO_VALUE_LOW);
		actFront.MoveServo(1,SERVO_VALUE_LOW);
		actFront.MoveServo(2,SERVO_VALUE_LOW);
		actBack.MoveServo(1,SERVO_VALUE_LOW);
		delay(250);
		actFront.SetPump(0,0);
		actFront.SetPump(1,0);
		actFront.SetPump(2,0);
		actBack.SetPump(1,0);
		delay(250);
		actFront.MoveServo(0,SERVO_VALUE_DROP);
		actFront.MoveServo(1,SERVO_VALUE_DROP);
		actFront.MoveServo(2,SERVO_VALUE_DROP);
		actBack.MoveServo(1,SERVO_VALUE_DROP);
		delay(250);
		actFront.MoveServo(0,SERVO_VALUE_HIGH);
		actFront.MoveServo(1,SERVO_VALUE_HIGH);
		actFront.MoveServo(2,SERVO_VALUE_HIGH);
		actBack.MoveServo(1,SERVO_VALUE_HIGH);

	}
    dspic.stop();
	dspic.setVar8(CODE_VAR_VERBOSE,0);
    dspic.stopThreadReception();
	puts("verbose set to 0");
    puts("exiting ...");
	lidar.stop();
    lidar.stopThreadDetection();
    web.stopThread();

    puts("exiting...");

	delay(200);
	
    return 0;
}

void debugAct(){

    wiringPiSetup();
    SPI spi(SPI_CHANNEL,SPI_SPEED); //initialise SPI
    /*A AJOUTER : FLUSH tous les slaves*/
    Actuators actFront(&spi,SPI_ID_ACT_FRONT), actBack(&spi,SPI_ID_ACT_BACK);

    DsPIC dspic;

    dspic.async_read(); //flush rx buffer

    dspic.setVar8(CODE_VAR_VERBOSE,1);
    puts("verbose set to 1");
    dspic.getVar(CODE_VAR_BAT);
    dspic.loadVarDspicFromFile("config.txt");
    dspic.initPos(1000,1500,3.14159);

    int choice = 0;
    while(choice != -1){
        std::cout << "Press -1 to exit" << std::endl;
        std::cout << "Select arm (0-5)" << std::endl;
        std::cout << "Press 10 to set goldenium arm" << std::endl;
        std::cout << "Press 11 to reset goldenium arm" << std::endl;
        std::cin >> choice;
        if(choice >= 0 && choice <= 5){
            int id = choice;
            std::cout << "Press 1 to start pump" << std::endl;
            std::cout << "Press 2 to stop pump" << std::endl;
            std::cout << "Press 3 to measure current" << std::endl;
            std::cout << "Press 4 to set servo" << std::endl;
            std::cout << "Press 5 to testMeasure" << std::endl;
            std::cin >> choice;
            switch(choice){
                case 1:
                    if(id < 3){
                        actFront.SetPump(id,1);
                    }
                    else{
                        actBack.SetPump(id-3,1);
                    }
                    break;
                case 2:
                    if(id < 3){
                        actFront.SetPump(id,0);
                    }
                    else{
                        actBack.SetPump(id-3,0);
                    }
                    break;
                case 3:
                    if(id < 3){
                        std::cout << "current : " << actFront.debugGetCurrent(id) << std::endl;
                    }
                    else{
                        std::cout << "current : " << actBack.debugGetCurrent(id-3) << std::endl;
                    }
                    break;
                case 4:
                    std::cout << "enter duty cycle" << std::endl;
                    int dutyCycle;
                    std::cin >> dutyCycle;
                    if(id < 3){
                        actFront.MoveServo(id,dutyCycle);
                    }
                    else{
                        actBack.MoveServo(id-3,dutyCycle);
                    }
                    break;
                case 5:
                    if(id < 3){
                        actFront.SetPump(id,1);
                        delay(2000);
                        std::cout << "current : " << actFront.debugGetCurrent(id) << std::endl;
                        std::cout << "currentFull : " << actFront.debugGetCurrentFull(id) << std::endl;
                        actFront.SetPump(id,0);
                    }
                    else{
                        actBack.SetPump(id-3,1);
                        delay(500);
                        std::cout << "current : " << actBack.debugGetCurrent(id-3) << std::endl;
                        std::cout << "currentFull : " << actBack.debugGetCurrentFull(id-3) << std::endl;
                        actBack.SetPump(id-3,0);
                    }
                    break;
                case 6:
                    if(id < 3){
                        std::cout << "color : " << actFront.debugGetColor(id) << std::endl;
                    }
                    else{
                        std::cout << "color : " << actBack.debugGetColor(id-3) << std::endl;
                    }
                    break;
            }

        }
        else if(choice == 10){
            dspic.setMotLin(1);
        }
        else if(choice == 11){
            dspic.setMotLin(0);
        }
        else if(choice == 20){
            dspic.motorVoltage(0,8);
        }
        else if(choice == 21){
            dspic.motorVoltage(0,0);
        }

    }
    dspic.setVar8(CODE_VAR_VERBOSE,0);
    puts("verbose set to 0");
}
void debugTestAllDelay(){
    SPI spi(SPI_CHANNEL,SPI_SPEED); //initialise SPI
    /*A AJOUTER : FLUSH tous les slaves*/
    Actuators actFront(&spi,SPI_ID_ACT_FRONT), actBack(&spi,SPI_ID_ACT_BACK);

    int valueH = 800;
    int valueL = 1600;
    int valueDrop = 1500;
    int valueMiddle = 1000;

    actFront.SetPump(0,1);
    actFront.MoveServo(0,valueL);
    delay(250);
    actFront.MoveServo(0,valueH);

    actFront.SetPump(1,1);
    actFront.MoveServo(1,valueL);
    delay(250);
    actFront.MoveServo(1,valueH);

    actFront.SetPump(2,1);
    actFront.MoveServo(2,valueL);
    delay(250);
    actFront.MoveServo(2,valueH);

    actBack.SetPump(0,1);
    actBack.MoveServo(0,valueL);
    delay(250);
    actBack.MoveServo(0,valueH);

    actBack.SetPump(1,1);
    actBack.MoveServo(1,valueL);
    delay(250);
    actBack.MoveServo(1,valueH);

    actBack.SetPump(2,1);
    actBack.MoveServo(2,valueL);
    delay(250);
    actBack.MoveServo(2,valueH);


    getchar();

    actFront.MoveServo(0,valueL);
    delay(250);
    actFront.SetPump(0,0);

    actFront.MoveServo(1,valueL);
    delay(250);
    actFront.SetPump(1,0);

    actFront.MoveServo(2,valueL);
    delay(250);
    actFront.SetPump(2,0);

    actBack.MoveServo(0,valueL);
    delay(250);
    actBack.SetPump(0,0);

    actBack.MoveServo(1,valueL);
    delay(250);
    actBack.SetPump(1,0);

    actBack.MoveServo(2,valueL);
    delay(250);
    actBack.SetPump(2,0);

    getchar();

    actFront.MoveServo(0,valueDrop);
    actFront.MoveServo(1,valueDrop);
    actFront.MoveServo(2,valueDrop);

    actBack.MoveServo(0,valueDrop);
    actBack.MoveServo(1,valueDrop);
    actBack.MoveServo(2,valueDrop);

    getchar();

    actFront.MoveServo(0,valueMiddle);
    actFront.MoveServo(1,valueMiddle);
    actFront.MoveServo(2,valueMiddle);

    actBack.MoveServo(0,valueMiddle);
    actBack.MoveServo(1,valueMiddle);
    actBack.MoveServo(2,valueMiddle);
}
void debugTestAllInstant(){
    SPI spi(SPI_CHANNEL,SPI_SPEED); //initialise SPI
    /*A AJOUTER : FLUSH tous les slaves*/
    Actuators actFront(&spi,SPI_ID_ACT_FRONT), actBack(&spi,SPI_ID_ACT_BACK);

    int valueH = 800;
    int valueL = 1600;
    int valueDrop = 1500;
    int valueMiddle = 1000;

    actFront.SetPump(0,1);
    actFront.SetPump(1,1);
    actFront.SetPump(2,1);

    actBack.SetPump(0,1);
    actBack.SetPump(1,1);
    actBack.SetPump(2,1);

    actFront.MoveServo(0,valueL);
    actFront.MoveServo(1,valueL);
    actFront.MoveServo(2,valueL);

    actBack.MoveServo(0,valueL);
    actBack.MoveServo(1,valueL);
    actBack.MoveServo(2,valueL);

    delay(250);


    actFront.MoveServo(0,valueH);
    actFront.MoveServo(1,valueH);
    actFront.MoveServo(2,valueH);

    actBack.MoveServo(0,valueH);
    actBack.MoveServo(1,valueH);
    actBack.MoveServo(2,valueH);

    getchar();

    actFront.MoveServo(0,valueL);
    actFront.MoveServo(1,valueL);
    actFront.MoveServo(2,valueL);

    actBack.MoveServo(0,valueL);
    actBack.MoveServo(1,valueL);
    actBack.MoveServo(2,valueL);


    delay(250);

    actFront.SetPump(0,0);
    actFront.SetPump(1,0);
    actFront.SetPump(2,0);

    actBack.SetPump(0,0);
    actBack.SetPump(1,0);
    actBack.SetPump(2,0);

    getchar();

    actFront.MoveServo(0,valueDrop);
    actFront.MoveServo(1,valueDrop);
    actFront.MoveServo(2,valueDrop);

    actBack.MoveServo(0,valueDrop);
    actBack.MoveServo(1,valueDrop);
    actBack.MoveServo(2,valueDrop);

    getchar();

    actFront.MoveServo(0,valueMiddle);
    actFront.MoveServo(1,valueMiddle);
    actFront.MoveServo(2,valueMiddle);

    actBack.MoveServo(0,valueMiddle);
    actBack.MoveServo(1,valueMiddle);
    actBack.MoveServo(2,valueMiddle);
}
void debugBN(){
    SPI spi(SPI_CHANNEL,SPI_SPEED); //initialise SPI
    /*A AJOUTER : FLUSH tous les slaves*/
    Actuators actFront(&spi,SPI_ID_ACT_FRONT), actBack(&spi,SPI_ID_ACT_BACK);

    //int valueH = 800;
    int valueL = 1600;
    int valueDrop = 1500;
    int valueMiddle = 1000;

    actBack.SetPump(2,1);
    delay(10);
    actBack.MoveServo(2,valueL);
    getchar();
    actBack.MoveServo(2,1000);

    getchar();
    actBack.MoveServo(2,valueL);
    delay(250);
    actBack.SetPump(2,0);
    delay(250);
    actBack.MoveServo(2,valueDrop);
    delay(250);
    actBack.MoveServo(2,valueMiddle);
}
void debugGoldenium(){
    DsPIC dspic;

    dspic.async_read(); //flush rx buffer

    std::cout << "Press enter to continue"  << std::endl;
    getchar();
    dspic.setVar8(CODE_VAR_VERBOSE,1);
    dspic.start();
    dspic.getVar(CODE_VAR_BAT);
    getchar();
    dspic.setMotLin(1);
    //dspic.motorVoltage(1,8);
    //dspic.getVar(CODE_VAR_RUPT);
    getchar();
    dspic.motorVoltage(0,10);
    getchar();
    dspic.setMotLin(0);
    getchar();
    dspic.setMotLin(1);
    getchar();
    dspic.motorVoltage(0,0);
    //dspic.motorVoltage(1,-8);
    getchar();
    dspic.motorVoltage(1,0);
    getchar();
    dspic.setMotLin(0);
    dspic.stop();
    dspic.setVar8(CODE_VAR_VERBOSE,0);
}
void reglageOdometrie(){

}
