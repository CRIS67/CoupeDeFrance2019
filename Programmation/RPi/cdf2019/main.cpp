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

//DStarGlobal 
int mapRows {10};  
int mapColumns {10};  
float km {0}; // variable for the D*

std::vector<std::vector<int>> mapVector; // the robot's map 

bool obstacleDetection {false}; 
bool pointReached {false}; 
 
Node startNode = {infinity,infinity,0,std::pair<int,int>(0,0)};
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
    /*A AJOUTER : FLUSH tous les slaves*/
    Actuators actFront(&spi,SPI_ID_ACT_FRONT), actBack(&spi,SPI_ID_ACT_BACK);
    Lidar lidar(&spi,SPI_ID_LIDAR);
	
    int valueH = 700;
    int valueL = 1600;
    int valueDrop = 1500;
    int valueMiddle = 1000;

    DsPIC dspic;
    pthread_t thread_print;

    dspic.async_read(); //flush rx buffer

    Web web(&dspic);
    web.startThread();

    int rc;
    //std::cout << "main() : creating thread, " << std::endl;
    rc = pthread_create(&thread_print, NULL, print, &web);

    if (rc) {
		std::cout << "Error:unable to create thread," << rc << std::endl;
		exit(-1);
    }

    getchar();
    dspic.setVar8(CODE_VAR_VERBOSE,1);
    puts("verbose set to 1");
    dspic.getVar(CODE_VAR_BAT);
    dspic.loadVarDspicFromFile("config.txt");
    //dspic.initPos(1000,1500,3.14159);
    //dspic.initPos(1000,1500,0);
    dspic.initPos(0,0,0);
    //dspic.initPos(1000,3000,-3.14159/2);
    std::cout << "Press enter to dspic.start() " << std::endl; 
    getchar();
    dspic.start();
    std::cout << "Press enter to start the D*" << std::endl; 
    getchar();

    /*=============Strategy  START===================*/
    
    std::vector<Node> strategyTour; 
    LK strategy(100); 
    strategy.readNodes(strategyTour); // reads the nodes from the entree.txt file  
    strategy.optimize(strategyTour); // optimize the tour 
    strategy.printSolution(); // debug 
    std::cout << "Initial distance " <<  heuristic.calculateDistance(toure) << std::endl; 
    std::cout << "Final distance " <<  heuristic.getDistance() << std::endl; 
    strategyTour = strategy.getSolution();  // We update the optimized tour 

    // Map Generation 
    generateMap(mapVector,mapRows,mapColumns); // generates empty map 
    createRectangle(4, 4, 5, 5, mapVector); // creates a 5x5 obstacle rectangle  at (4,4) 
    printMap(mapRows, mapColumns, mapVector);

    /* For all actions we do a D* run */

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

      std::vector<Node> completePath = getPath(mapVector, knownNodes, startNode, goalNode); // get the hole path 
      std::vector<Node> simplifiedPath = pathTreatment(completePath);
      simplifiedPath.push_back(goalNode); // we need to add the last node manually :(
      printPath(simplifiedPath,mapVector); 

      int counter=0; 

      /* Dstar Loop*/
      while(startNode.coord != goalNode.coord){

          if(startNode.costG == infinity){
              std::cerr << "NOT KNOWN PATH" << std::endl;
              break;
          }

          //startNode = bestNode(startNode, knownNodes); // we "move" the robot
          startNode = simplifiedPath.at(counter); 
          counter++;
          findPath(mapVector,knownNodes,startNode,goalNode); // prints the path in the terminal 

          int xSetpoint = startNode.coord.first *30; 
          int ySetpoint = startNode.coord.second *30; 
          dspic.go(xSetpoint, ySetpoint,0,0); // we move the robot to the next point

          // Wait until the robot reaches the point
          /*
           while(!dspic.arrived){
               delay(50);  //wait before asking so the dspic can start the movement /  and don't SPAM the UART channel
               dspic.getVar(CODE_VAR_ARRIVED); //send a request to update the arrived variable
               delay(50); 
           }*/
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
          std::cout << "Press enter to continue to the next point" << std::endl; 
          getchar();
      }

      /*=============DStarImplementation END===================*/
      strategyTour.erease(strategyTour.begin()+i); // We remove the action

 
    }


        /*=============Strategy  END ===================*/

    dspic.stop();
    dspic.setVar8(CODE_VAR_VERBOSE,0);
    puts("verbose set to 0");
    puts("exiting ...");

    return 0;
}

void *print(void *ptr) {
   Web *w = (Web*)ptr;
   DsPIC *dspic = w->dspic;
   while(1){
        std::vector<uint8_t> msg = dspic->readMsg();
        uint8_t checksum = 0;
        for(unsigned int i = 0; i < msg.size() - 1; i++){
            checksum += msg[i];
        }
        if(checksum != msg[msg.size() - 1]){
            std::cout << "CHECKSUM ERROR !" << std::endl;
			std::cout << "CE dec :";
            for(unsigned int i = 0; i < msg.size(); i++){
                std::cout << " & [" << i << "] = " << (int)msg[i];
            }
			std::cout << std::endl << "CE char :";
			
            for(unsigned int i = 0; i < msg.size(); i++){
                if(msg[i] > 31 && msg[i] < 127)
                    std::cout << msg[i];
            }
        }
        else{
            if(msg.size() > 1){
                switch(msg[1]){
                    case 0 :    //error

                        break;
                    case TX_CODE_VAR :    //variable
                        switch(msg[2]){
							case CODE_VAR_BAT :{
                                if(msg.size() > 6){
                                    float vbat;
									float *ptr = &vbat;
									uint8_t *ptrChar = (uint8_t*)ptr;
									for(int i = 0; i < 4; i++){
										ptrChar[i] = msg[3+i];
									}
									std::cout << "received from DsPIC : VBAT = " << vbat << std::endl;
									dspic->bat = vbat;
                                }
							break;
							}
                            case CODE_VAR_X :
                                if(msg.size() > 4){
                                    dspic->x = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : x = " << dspic->x << std::endl;
                                }
                                break;
                            case CODE_VAR_Y :
                                if(msg.size() > 4){
                                    dspic->y = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : y = " << dspic->y << std::endl;
                                }
                                break;
                            case CODE_VAR_T :
                                if(msg.size() > 4){
                                    dspic->t = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : t = " << dspic->t << " & H = " << (int)msg[3] << " & L = " << (int)msg[4] << std::endl;
                                }
                            case CODE_VAR_X_LD :
                                if(msg.size() > 8){
                                    double x_ld;
                                    double *ptr = &x_ld;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    /*std::cout.precision(11);
                                    std::cout << "received from DsPIC : x_ld = " << x_ld << std::endl;
                                    std::cout.precision(6);*/
                                    //dspic->bat = vbat;
                                    dspic->x_ld = x_ld;
                                }
                                break;
                            case CODE_VAR_Y_LD :
                                if(msg.size() > 8){
                                    double y_ld;
                                    double *ptr = &y_ld;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    dspic->y_ld = y_ld;
                                    /*std::cout.precision(11);
                                    std::cout << "received from DsPIC : y_ld = " << y_ld << std::endl;
                                    std::cout.precision(6);*/
                                    //dspic->bat = vbat;
                                }
                                break;
                            case CODE_VAR_T_LD :
                                if(msg.size() > 8){
                                    double t_ld;
                                    double *ptr = &t_ld;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    dspic->t_ld = t_ld;
                                    /*std::cout.precision(11);
                                    std::cout << "received from DsPIC : t_ld = " << t_ld << std::endl;
                                    std::cout.precision(6);*/
                                    //dspic->bat = vbat;
                                }
                                break;
                            case CODE_VAR_RUPT :
                                if(msg.size() > 4){
                                    //printf("msg[3] = %d / 0x01 = %d / msg[3] & 0x01 = %d\n",msg[3],0x01,msg[3] & 0x01);
                                    //printf("msg[3] = %d / 0x02 = %d / msg[3] & 0x02 = %d\n",msg[3],0x02,msg[3] & 0x02);
                                    //printf("msg[3] = %d / 0x04 = %d / msg[3] & 0x04 = %d\n",msg[3],0x04,msg[3] & 0x04);
                                    //printf("msg[3] = %d / 0x08 = %d / msg[3] & 0x08 = %d\n",msg[3],0x08,msg[3] & 0x08);
                                    dspic->rupt.ass0 = msg[3] & 0x01;
                                    dspic->rupt.ass1 = (msg[3] & 0x02) >> 1;
                                    dspic->rupt.ass2 = (msg[3] & 0x04) >> 2;
                                    dspic->rupt.ass3 = (msg[3] & 0x08) >> 3;
                                    dspic->rupt.act0 = (msg[3] & 0x10) >> 4;
                                    dspic->rupt.act1 = (msg[3] & 0x20) >> 5;
                                    dspic->rupt.act2 = (msg[3] & 0x40) >> 6;
                                    dspic->rupt.act3 = (msg[3] & 0x80) >> 7;
                                    dspic->rupt.act4 = msg[4] & 0x01;
                                    dspic->rupt.act5 = (msg[4] & 0x02) >> 1;
                                    //std::cout << "received from DsPIC : rupt = H:" << (int)msg[3] << " & L: " << (int)msg[4] << " & " << dspic->rupt.ass0 << "/" << dspic->rupt.ass1 << "/" << dspic->rupt.ass2 << "/" << dspic->rupt.ass3 << "/" << dspic->rupt.act0 << "/" << dspic->rupt.act1 << "/" << dspic->rupt.act2 << "/" << dspic->rupt.act3 << "/" << dspic->rupt.act4 << "/" << dspic->rupt.act5 << "/"  << std::endl;
                                }
                                break;
                            case 100 :
                                if(msg.size() > 4){
                                    dspic->US[0] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[0] = " << dspic->US[0] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 101 :
                                if(msg.size() > 4){
                                    dspic->US[1] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[1] = " << dspic->US[1] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 102 :
                                if(msg.size() > 4){
                                    dspic->US[2] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[2] = " << dspic->US[2] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 103 :
                                if(msg.size() > 4){
                                    dspic->US[3] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[3] = " << dspic->US[3] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 104 :
                                if(msg.size() > 4){
                                    dspic->US[4] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[4] = " << dspic->US[4] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 105 :
                                if(msg.size() > 4){
                                    dspic->US[5] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[5] = " << dspic->US[5] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
							case CODE_VAR_P_SPEED_L :
								w->dspic->pidSpeedLeft.Kp = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;
							case CODE_VAR_I_SPEED_L :
								w->dspic->pidSpeedLeft.Ki = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;
							case CODE_VAR_D_SPEED_L :
								w->dspic->pidSpeedLeft.Kd = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;

							case CODE_VAR_P_SPEED_R :
								w->dspic->pidSpeedRight.Kp = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;
							case CODE_VAR_I_SPEED_R :
								w->dspic->pidSpeedRight.Ki = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;
							case CODE_VAR_D_SPEED_R :
								w->dspic->pidSpeedRight.Kd = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;

							case CODE_VAR_P_DISTANCE :
								w->dspic->pidDistance.Kp = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;
							case CODE_VAR_I_DISTANCE :
								w->dspic->pidDistance.Ki = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;
							case CODE_VAR_D_DISTANCE :
								w->dspic->pidDistance.Kd = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;

							case CODE_VAR_P_ANGLE :
								w->dspic->pidAngle.Kp = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;
							case CODE_VAR_I_ANGLE :
								w->dspic->pidAngle.Ki = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								break;
							case CODE_VAR_D_ANGLE :
								w->dspic->pidAngle.Kd = ((uint32_t)msg[3] << 24) + ((uint32_t)msg[4] << 16) + ((uint32_t)msg[5] << 8) + msg[6];
								w->dspic->isPIDUpdated = true;
								break;
                            case CODE_VAR_COEF_DISSYMETRY_LD:
                                if(msg.size() > 8){
                                    double var;
                                    double *ptr = &var;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    std::cout << "received from DsPIC : coef_Dissymetry_ld = " << var << std::endl;
                                }
                                break;
                            case CODE_VAR_MM_PER_TICKS_LD:
                                if(msg.size() > 8){
                                    double var;
                                    double *ptr = &var;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    std::cout << "received from DsPIC : mm_per_ticks_ld = " << var << std::endl;
                                }
                                break;
                            case CODE_VAR_RAD_PER_TICKS_LD:
                                if(msg.size() > 8){
                                    double var;
                                    double *ptr = &var;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    std::cout << "received from DsPIC : rad_per_ticks_ld = " << var << std::endl;
                                }
                                break;
                            default :
                                std::cout << "Received wrong variable code from DsPIC : " << (int)msg[2] << std::endl;
                                break;

                        }
                        break;
                    case TX_CODE_LOG :{    //log
                        std::string s;
                        for(unsigned int i = 2; i < msg.size() - 1; i++)
                            s += msg[i];
                        std::cout << "Received log from DsPIC : " << s << std::endl;
                        dspic->logs.push(s);
                        //w->sendMsg("l=" + s);
                        break;
                    }
                    case TX_CODE_PLOT :{    //plot
                        uint8_t id = msg[2];
                        //std::cout << "plot id = " << id << "/" << (int)msg[2] << std::endl;
                        uint32_t x = (msg[3] << 24) + (msg[4] << 16) + (msg[5] << 8) + msg[6];
                        int32_t y = (msg[7] << 24) + (msg[8] << 16) + (msg[9] << 8) + msg[10];
                        point p = {id, x, y};
                        dspic->plots.push(p);
                        break;
                    }
                    default :
                        std::cout << "Received wrong message code from DsPIC : " << msg[1] << std::endl;
                        break;
                }
            }
        }


            //std::cout << "DsPIC >>" << s << std::endl;

	   //delay(100);
   }
   std::cout << "Hello World!" << std::endl;
   pthread_exit(NULL);
}
void debugAct(){

    wiringPiSetup();
    SPI spi(SPI_CHANNEL,SPI_SPEED); //initialise SPI
    /*A AJOUTER : FLUSH tous les slaves*/
    Actuators actFront(&spi,SPI_ID_ACT_FRONT), actBack(&spi,SPI_ID_ACT_BACK);

    DsPIC dspic;
    pthread_t thread_print;

    dspic.async_read(); //flush rx buffer

    Web web(&dspic);
    web.startThread();

    int rc;
    //std::cout << "main() : creating thread, " << std::endl;
    rc = pthread_create(&thread_print, NULL, print, &web);

    if (rc) {
        std::cout << "Error:unable to create thread," << rc << std::endl;
        exit(-1);
    }

    getchar();
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
            std::cout << "Press 3 to set servo" << std::endl;
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
    pthread_t thread_print;

    dspic.async_read(); //flush rx buffer

    Web web(&dspic);
    web.startThread();

    int rc;
    //std::cout << "main() : creating thread, " << std::endl;
    rc = pthread_create(&thread_print, NULL, print, &web);

    if (rc) {
        std::cout << "Error:unable to create thread," << rc << std::endl;
        exit(-1);
    }
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
