CDF: web.o dspic.o SPI.o lidar.o actuators.o hmi.o main.o  
	g++ -o cdf web.o dspic.o SPI.o lidar.o actuators.o hmi.o main.o -Wall -lwiringPi -pthread 

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

hmi.o: hmi.cpp 
	g++ -o hmi.o -c hmi.cpp -Wall -lwiringPi -pthread 

main.o: main.cpp web.hpp dspic.hpp SPI.hpp lidar.hpp actuators.hpp hmi.hpp
	g++ -o main.o -c main.cpp -Wall -lwiringPi -pthread 
