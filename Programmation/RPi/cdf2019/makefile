CDF: web.o dspic.o SPI.o lidar.o actuators.o mapGeneration.o dStarLite.o trajectoryHandle.o LK.o main.o  
	g++ -o cdf web.o dspic.o SPI.o lidar.o actuators.o mapGeneration.o dStarLite.o trajectoryHandle.o LK.o main.o -Wall -lwiringPi -pthread 

web.o: web.cpp
	g++ -o web.o -c web.cpp -Wall -lwiringPi -pthread 

dspic.o: dspic.cpp 
	g++ -o dspic.o -c dspic.cpp -Wall -lwiringPi -pthread

SPI.o: SPI.cpp 
	g++ -o SPI.o -c SPI.cpp -Wall -lwiringPi -pthread 
	
lidar.o: lidar.cpp 
	g++ -o lidar.o -c lidar.cpp -Wall -lwiringPi -pthread 
	
actuators.o: actuators.cpp 
	g++ -o actuators.o -c actuators.cpp -Wall -lwiringPi -pthread

mapGeneration.o : mapGeneration.cpp 
	g++ -o mapGeneration.o -c mapGeneration.cpp -Wall -lwiringPi -pthread

dStarLite.o : dStarLite.cpp 
	g++ -o dStarLite.o -c dStarLite.cpp -Wall -lwiringPi -pthread

trajectoryHandle.o : trajectoryHandle.cpp 
	g++ -o trajectoryHandle.o -c trajectoryHandle.cpp -Wall -lwiringPi -pthread

LK.o: LK.cpp
	g++   -o LK.o -c LK.cpp -Wall -Wextra -pedantic -ansi -std=c++11

main.o: main.cpp web.hpp dspic.hpp SPI.hpp lidar.hpp actuators.hpp mapGeneration.hpp dStarLite.hpp trajectoryHandle.hpp LK.h
	g++ -o main.o -c main.cpp -Wall -lwiringPi -pthread 
