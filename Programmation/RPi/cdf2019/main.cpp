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

#define MAP_MM_PER_ARRAY_ELEMENT 	10
#define SIZE_ENNEMY					20	//size of ennemy /2 4 element in array map -> (here 40cm)WARNING depends of map resolution
//DStarGlobal 
int mapRows {200};  
int mapColumns {300};  
float km {0}; // variable for the D*

std::vector<std::vector<int>> mapVector; // the robot's map 

bool obstacleDetection {false}; 
bool pointReached {false}; 
 
Node startNode = {infinity,infinity,0,std::pair<int,int>(100,250)};
Node goalNode  = {infinity,0,0,std::pair<int,int>(9,9), false};

priorityList uList; // priority List
mappedNodes knownNodes; // node the robot can see


const int enemyWidth = 500; // The distance the enemy takes place on the map, it is represented as a square 

std::vector<Node> simplifiedPath; 
std::vector<Node> completePath; 

// Strategy include 
#include "LK.h" 


void *print(void *ptr);
void debugAct();
void debugTestAllDelay();
void debugTestAllInstant();
void debugBN();
void debugGoldenium();
int main()
{
    wiringPiSetup();
	
    SPI spi(SPI_CHANNEL,SPI_SPEED); //initialise SPI
    Actuators actFront(&spi,SPI_ID_ACT_FRONT), actBack(&spi,SPI_ID_ACT_BACK);
	
	DsPIC dspic;
    dspic.async_read(); //flush rx buffer
	
    dspic.setVar8(CODE_VAR_VERBOSE,0);
    dspic.stop();
	delay(50);
    dspic.async_read(); //flush rx buffer
	
    
    Web web(&dspic);
    web.startThread();

    Lidar lidar(&spi, SPI_ID_LIDAR, &web);
    lidar.setSpeed(150);
	lidar.stop();
	std::cout << ">DEBUG : stop dspic & lidar" << std::endl;
	std::cout << "Press <ENTER> to start" << std::endl;
	getchar();
    lidar.start();
    lidar.startThreadDetection();

    puts("Hello human ! I, your fervent robot, am initialised. Press <ENTER> to continue.");

    getchar();
    dspic.setVar8(CODE_VAR_VERBOSE,1);
    puts("verbose set to 1");
    dspic.getVar(CODE_VAR_BAT);
    dspic.loadVarDspicFromFile("config.txt");
    //dspic.initPos(1000,1500,3.14159);
    //dspic.initPos(1200,2550,-3.14159/2);
    dspic.initPos(startNode.coord.first*10,startNode.coord.second*10, -3.14159/2);
    //dspic.initPos(1000,1500,-3.14159/2);
    //dspic.initPos(0,0,0);
    //dspic.initPos(1000,3000,-3.14159/2);
	
    std::cout << "Press enter to dspic.start() " << std::endl; 
    getchar();
    dspic.start(); 
    std::cout << "Press enter to start the D*" << std::endl; 
    getchar(); 

    /*=============Strategy  START===================*/
    
    std::vector<Node> strategyTour; // Contains the nodes we are going to visit 
    LK strategy(100); 
    strategy.readNodes(strategyTour); // reads the nodes from the entree.txt file  
    strategy.optimize(strategyTour); // optimize the tour 
    strategy.printSolution(); // debug 
    std::cout << "Initial distance " <<  strategy.calculateDistance(strategyTour) << std::endl; 
    std::cout << "Final distance " <<  strategy.getDistance() << std::endl; 
    strategyTour = strategy.getSolution();  // We update the optimized tour 

    // Map Generation 
    generateMap(mapVector,mapRows,mapColumns); // generates empty map 
    createRectangle(0,0,30,300, mapVector); // creates a 400x2000 obstacle rectangle  at (1600,0) 
    createRectangle(170,0,25,300, mapVector); // creates a 400x2000 obstacle rectangle  at (1600,0) 
    //createRectangle(90,140,20,20, mapVector); // creates a 400x2000 obstacle rectangle  at (1600,0) 

   // /*Ensemble palets*/
   // createRectangle(90,85,30,30,mapVector); 
   // createRectangle(90,185,30,30,mapVector); 
   // 
   // /* Palets*/
   // createRectangle(41,41,7,7,mapVector); 
   // createRectangle(71,41,7,7,mapVector); 
   // createRectangle(103,41,7,7,mapVector); 
   // createRectangle(41,246,7,7,mapVector); 
   // createRectangle(71,246,7,7,mapVector); 
   // createRectangle(103,246,7,7,mapVector); 
     
    std::cout << "MAP GENERATED" << std::endl; 
    //printMap(mapRows, mapColumns, mapVector);

	
	/*save map in a file*/
	std::ofstream file;
	file.open("out_map.txt");
	for(unsigned int i = 0; i < mapRows;i++){
		for(unsigned int j = 0; j < mapColumns; j++){
			file << mapVector.at(i).at(j) << " ";
		}
		file << std::endl;
	}
	file.close();
	
    for(uint i = 0; i< strategyTour.size(); i++)
    {
		/*=============DStarImplementation START===================*/

		goalNode.coord = strategyTour.at(i).coord; // Coordinates of the next action  
		//DStarLite first run
		Node lastNode = startNode;
		initialize(mapVector, knownNodes, uList, startNode, goalNode);
		goalNode = knownNodes.at(goalNode.coord);
		computeShortestPath(uList, knownNodes, startNode.coord, goalNode);
		startNode = knownNodes.at(startNode.coord); // we update the start node
		goalNode = knownNodes.at(goalNode.coord); // we update the goal node
		std::cout << "Start Node coord " << startNode.coord.first << " " << startNode.coord.second << std::endl; 
		std::cout << " Goal Node coord " << goalNode.coord.first << " " << goalNode.coord.second << std::endl; 

		std::vector<Node> completePath = getPath(mapVector, knownNodes, startNode, goalNode); // get the whole path 
	  
		std::cout << "getPath ended" << std::endl;
		/* <modifié*/
		
		std::vector<Node> tempSimplifiedPath = pathTreatment(completePath);
		tempSimplifiedPath.push_back(goalNode); // we need to add the last node manually :(
		std::vector<Node> simplifiedPath = optimizePath(mapVector,tempSimplifiedPath, startNode);
		
		/*std::cout << "tempSimplified path : " << std::endl;
		for(unsigned int i = 0 ; i < tempSimplifiedPath.size(); i++){
			std::cout << i << " : (" << tempSimplifiedPath.at(i).coord.first << " ; "  << tempSimplifiedPath.at(i).coord.second << ")" << std::endl;
		}
		
		std::cout << "optimized path" << std::endl;
		for(unsigned int i = 0 ; i < simplifiedPath.size(); i++){
			std::cout << i << " : (" << simplifiedPath.at(i).coord.first << " ; "  << tempSimplifiedPath.at(i).coord.second << ")" << std::endl;
		}*/
		/*modifié>*/
		/*<original*/
		
		/*std::vector<Node> simplifiedPath = pathTreatment(completePath);
		
		//std::cout << "debug2" << std::endl;
		simplifiedPath.push_back(goalNode); // we need to add the last node manually :(
		//std::cout << "debug3" << std::endl;
		*/
		/*original>*/
		 

		int counter=0; 
		unsigned int nb = 0;
		/* Dstar Loop*/
		while(startNode.coord != goalNode.coord){

			if(startNode.costG == infinity){
				std::cerr << "NOT KNOWN PATH" << std::endl;
				break;
			}

			//startNode = bestNode(startNode, knownNodes); // we "move" the robot
			startNode = simplifiedPath.at(counter); 
			counter++;
			std::cout << "PRINTING PATH" << std::endl; 
			//findPath(mapVector,knownNodes,startNode,goalNode); // prints the path in the terminal 

			int xSetpoint = startNode.coord.first * MAP_MM_PER_ARRAY_ELEMENT; 
			int ySetpoint = startNode.coord.second * MAP_MM_PER_ARRAY_ELEMENT; 
			std::cout << "send go " << xSetpoint << " ; " << ySetpoint << std::endl;
			dspic.go(xSetpoint, ySetpoint,0,0); // we move the robot to the next point
			//delay(200);  //wait before asking so the dspic can start the movement /  and don't SPAM the UART channel

			// Wait until the robot reaches the point
          
			//while(!dspic.getArrived()){
			/*while( (dspic.getX() - xSetpoint)*(dspic.getX() - xSetpoint) + (dspic.getY() - ySetpoint)*(dspic.getY() - ySetpoint) > 400){
               delay(50);  //wait before asking so the dspic can start the movement /  and don't SPAM the UART channel
               dspic.getVar(CODE_VAR_ARRIVED); //send a request to update the arrived variable
               delay(50); 
			}*/
			/*while((dspic.getX() - xSetpoint)*(dspic.getX() - xSetpoint) + (dspic.getY() - ySetpoint)*(dspic.getY() - ySetpoint) > 400){
				delay(50);
			}*/
			
			lidar.setFillBuffer(true);	//keep track of the points detected by lidar in a member buffer
			while((dspic.getX() - xSetpoint)*(dspic.getX() - xSetpoint) + (dspic.getY() - ySetpoint)*(dspic.getY() - ySetpoint) > 400){
				//get lidar points
				std::queue<pointFloat2d> points = lidar.getAndClearDetectedPoints();	//get detected points and clear internal buffer
				std::queue<pointFloat2d> savePoints = points;
				unsigned int nbPoints = points.size();
				//std::cout << "nbPoints" << nbPoints << std::endl;
				//add on binary map
				std::vector<std::vector<int>> newMap(mapVector);
				for(unsigned int i = 0; i < nbPoints;i++){
					pointFloat2d p = points.front();
					points.pop();
					unsigned int x = (unsigned int)round(p.x / MAP_MM_PER_ARRAY_ELEMENT);
					unsigned int y = (unsigned int)round(p.y / MAP_MM_PER_ARRAY_ELEMENT);
					//std::cout << "(" << x << " ; " << y << ") ";
					for(unsigned int j = x - SIZE_ENNEMY; j < x + SIZE_ENNEMY;j++){
						for(unsigned int k = y - SIZE_ENNEMY; k < y + SIZE_ENNEMY;k++){
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
				nRobot.coord.first = (int)(dspic.getX()/10);
				nRobot.coord.second = (int)(dspic.getY()/10);
				bool obstacleDetection = detectCollision(newMap,simplifiedPath, nRobot);	//ATTENTION à changer plus tard pour ne pas prendre en compte la partie du trajet déjà fait
				//bool obstacleDetection = detectCollisionLine(100,50,100,250,newMap);	//ATTENTION à changer plus tard pour ne pas prendre en compte la partie du trajet déjà fait
				if(obstacleDetection){
					
					std::vector<std::vector<int>> augmentedMap(mapVector);
					for(unsigned int i = 0; i < nbPoints;i++){
						pointFloat2d p = savePoints.front();
						savePoints.pop();
						unsigned int x = (unsigned int)round(p.x / MAP_MM_PER_ARRAY_ELEMENT);
						unsigned int y = (unsigned int)round(p.y / MAP_MM_PER_ARRAY_ELEMENT);
						//std::cout << "(" << x << " ; " << y << ") ";
						for(unsigned int j = x - (SIZE_ENNEMY+20); j < x + (SIZE_ENNEMY+20);j++){
							for(unsigned int k = y - (SIZE_ENNEMY+20); k < y + (SIZE_ENNEMY+20);k++){
								if(j >= 0 && j < mapRows && k >= 0 && k < mapColumns){
									augmentedMap.at(j).at(k) = 1;
									//std::cout << "(" << j << " ; " << k << ") ";
								}
							}
						}
						//std::cout << std::endl;
					}
				
					startNode.coord.first = nRobot.coord.first;
					startNode.coord.second = nRobot.coord.second;
					std::cout << " WARNING : collision !" << std::endl;
					km = km + distance2(lastNode, startNode);
					lastNode = startNode;
					updateMap(knownNodes, augmentedMap, uList, startNode.coord, goalNode); // we update all the changed nodes
					std::cout << "computing new path" << std::endl;
					computeShortestPath(uList, knownNodes, startNode.coord, goalNode);
					std::cout << "new path found ! =)" << std::endl;
					startNode = knownNodes.at(startNode.coord); // we update the start node
					goalNode = knownNodes.at(goalNode.coord);	
					std::cout << "debug1" << std::endl;
					completePath = getPath(augmentedMap, knownNodes, startNode, goalNode); // get the whole path 	
					std::cout << "debug2" << std::endl;
					tempSimplifiedPath = pathTreatment(completePath);
					std::cout << "debug2.1" << std::endl;
					tempSimplifiedPath.push_back(goalNode); // we need to add the last node manually :(
					std::cout << "debug2.2" << std::endl;
					simplifiedPath = optimizePath(augmentedMap,tempSimplifiedPath, startNode);
					std::cout << "debug2.3" << std::endl;	
					counter=0;
					std::cout << "debug3" << std::endl;
					break;
				}
				//std::cout << "collision = " << obstacleDetection << std::endl;
				//std::cout << "collision = " << detectCollisionLine(100,100,100,250,newMap) << std::endl;
				std::ofstream file;
				std::ostringstream ss;
				ss << "out_map" << nb++ << ".txt";
				file.open(ss.str());
				for(unsigned int i = 0; i < mapRows;i++){
					for(unsigned int j = 0; j < mapColumns; j++){
						file << newMap.at(i).at(j) << " ";
					}
					file << std::endl;
				}
				file.close();	
				//printMap(mapRows,mapColumns,newMap);
				delay(50);
			}
			lidar.setFillBuffer(false);	//stop keeping lidar points to avoid running out of memory
			
          /*
          while(!pointReached)
          {
              
              TO BE IMPLEMENTED 
              - getPointReached() from the dsPic
              - readSensorValues() 
              - obstcaleDetection = sensorTreatment() -> determine if we have to update the map 
              
            if(obstacleDetection)
              {
                  km = km + distance2(lastNode, startNode);
                  lastNode = startNode;
                  updateMap(knownNodes, mapVector, uList, startNode.coord, goalNode); // we update all the changed nodes
                  computeShortestPath(uList, knownNodes, startNode.coord, goalNode);
                  startNode = knownNodes.at(startNode.coord); // we update the start node
                  goalNode = knownNodes.at(goalNode.coord);

                 
                  TO BE IMPLEMENTED 
                  - simplifiedPath = pathTreatment(getPath()) set critical points to go to 
                  - create a vector with those points and update them if changes in the map 
                  

              }
          }*/

          // Debug 
          std::cout << "Press nothing to continue to the next point" << std::endl; 
          //getchar();
      }

      /*=============DStarImplementation END===================*/
      startNode = goalNode; 
      std::cout << "Press nothing for next action"  << std::endl; 
      //getchar(); 
      //strategyTour.erase(strategyTour.begin()+i); // We remove the action
    }


    /*=============Strategy  END ===================*/

    lidar.stopThreadDetection();
	lidar.stop();
    dspic.setVar8(CODE_VAR_VERBOSE,0);
	dspic.stop();
    puts("verbose set to 0");
    puts("exiting ...");

	delay(50);
	
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
