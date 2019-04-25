#include "actuators.hpp"
int Send(int in) {
	std::cout << "entree = " << in << std::endl;
	unsigned char buffer[100];
	buffer[0] = (unsigned char)in;
	wiringPiSPIDataRW(CHANNEL, buffer, 1);
	std::cout << "reponse = " << (int)buffer[0] << std::endl;
	delayMicroseconds(SPI_DELAY);
	return (int)buffer[0];
}
Actuators::Actuators(int fd_SPI){
    m_fd = fd_SPI;
}
Actuators::~Actuators(){

}
/************************************
*
* nom de la fonction : MoveServoExtr
*
* entrée : nb_bras, numéro du bras de 0 à 2
*
*          pos, position du bras extrême de 0 à 1 (tout en bas)
*
* tâche effectée : bouge le bras à la position indiquée
*
************************************/
void Actuators::MoveServoExtr(int nb_bras, int pos){
	if(nb_bras < 0 || nb_bras > 2) {
		std::cout << "erreur nb_bras" << std::endl;
	} else {
		if(pos < 0 || pos > 1) {
			std::cout << "erreur pos" << std::endl;
		} else {
			nb_bras += 4;
			Send(3);
			Send(nb_bras);
			Send(pos);
			Send(3+nb_bras+pos);
		}
	}
}
/************************************
*
* nom de la fonction : MoveServo
*
* entrée : nb_bras, numéro du bras de 0 à 2
*
*          pos, position du bras de 700 à 1600 (tout en bas)
*
* tâche effectée : bouge le bras à la position indiquée
*
************************************/
void Actuators::MoveServo(int nb_bras, int pos){
	if(nb_bras < 0 || nb_bras > 2) {
		std::cout << "erreur nb_bras" << std::endl;
	} else {
		if(pos < 700 || pos > 1600) {
			std::cout << "erreur pos" << std::endl;
		} else {
			nb_bras += 7;
			int lsb = pos%256, msb = (int)(pos/256);
			Send(4);
			Send(nb_bras);
			Send(msb);
			Send(lsb);
			Send(4+nb_bras+lsb+msb);
		}
	}
}
/************************************
* nom de la fonction : SetPump
*
* entrée : nb_bras, numéro du bras de 0 à 2
*
*          state, état du bras de 0 à 1 (allumée)
*
* tâche effectée : allume ou éteint la pompe
*
************************************/
void Actuators::SetPump(int nb_bras, int state){
	if(nb_bras < 0 || nb_bras > 2) {
		std::cout << "erreur nb_bras" << std::endl;
	} else {
		if(state < 0 || state > 1) {
			std::cout << "erreur state" << std::endl;
		} else {
			nb_bras ++;
			Send(3);
			Send(nb_bras);
			Send(state);
			Send(3+nb_bras+state);
		}
	}
}
/************************************
* nom de la fonction : GetColor
*
* entrée : nb_bras, numéro du bras de 0 à 2
*
* tâche effectée : demande une lecture de la couleur
*
************************************/
void Actuators::GetColor(int nb_bras){
	if(nb_bras < 0 || nb_bras > 2) {
		std::cout << "erreur nb_bras" << std::endl;
	} else {
		nb_bras += 10;
		Send(2);
		Send(nb_bras);
		Send(2+nb_bras);
	}
}
/************************************
* nom de la fonction : GetCurrent
*
* entrée : nb_bras, numéro du bras de 0 à 2
*
* tâche effectée : demande une mesure de courant
*
************************************/
void Actuators::GetCurrent(int nb_bras){
	if(nb_bras < 0 || nb_bras > 2) {
		std::cout << "erreur nb_bras" << std::endl;
	} else {
		nb_bras += 13;
		Send(2);
		Send(nb_bras);
		Send(2+nb_bras);
	}
}
/************************************
* nom de la fonction : Launchtest
*
* tâche effectée : teste un bras
*
************************************/
void Actuators::Launchtest() {
	//InitSpi();
	int i = -1, res = 0;
	char in = 'a';
	std::cout << "numero du bras ? (0/1/2)" << std::endl;
	while(i < 0 || i > 2) {
		std::cin >> i;
	}
	MoveServoExtr(i, 0);
	delay(1000);
	std::cout << "placer un palet de couleur ? (R/G/B)" << std::endl;
	while(in != 'R' && in != 'G' && in != 'B') {
		std::cin >> in;
	}
	MoveServo(i,1600);
	delay(2000);
	SetPump(i, 1);
	delay(1000);
	GetCurrent(i);
	//get color
	Send(2);
	Send(i+10);
	res = Send(12+i);
	if(res == 2) {
		std::cout << "pump succes" << std::endl;
	} else {
		if(res == 1) {
			std::cout << "pump empty" << std::endl;
		} else {
			std::cout << "pump error" << std::endl;
		}
	}
	//pump off
	Send(3);
	Send(i+1);
	Send(0);
	res = Send(4+i);
	if((res == 1 && in == 'R') || (res == 2 && in == 'G') || (res == 3 && in == 'B')) {
		std::cout << "succes" << std::endl;
	} else {
		std::cout << "fail" << std::endl;
	}
	MoveServoExtr(i, 0);
}