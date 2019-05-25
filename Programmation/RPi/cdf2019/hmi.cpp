#include "hmi.hpp"

HMI::HMI(SPI *pSpi,uint8_t id){
    m_pSpi = pSpi;
	m_id = id;

	flush(255);
	//ajouter ping();
}
HMI::~HMI(){

}

void HMI::SendManual(void) {
	unsigned char buffer[100];
	int in = 0;
	while(in != -1) {
		std::cout << ">";
		std::cin >> in;
		if(in > -1 && in < 256) {
			buffer[0] = (unsigned char)in;
			wiringPiSPIDataRW(SPI_CHANNEL, buffer, 1);
			std::cout << "reponse = " << (int)buffer[0] << std::endl;
			delay(10);
		} else {
			in = -1;
		}
	}
}
int HMI::Send(int in){
	unsigned char buffer[1];
	buffer[0] = (unsigned char)in;
	wiringPiSPIDataRW(SPI_CHANNEL, buffer, 1);
	std::cout << "entree = " << in << "   /   " << " reponse = " << (int)buffer[0] << std::endl;
	//delay(SPI_DELAY_MS);
	delayMicroseconds(SPI_DELAY_US);
	return (int)buffer[0];

}
void HMI::SetPrgm(int in, std::string txt) {
	m_pSpi->lock();
	if(m_pSpi->getSlaveId() != m_id){	
		m_pSpi->setSlave(m_id);		//change Chip select
	}
	unsigned int i;
	int check = 0;
	char txt_char[txt.size()+1];
	strcpy(txt_char, txt.c_str());
	if(txt.size() < 16) {
		Send(2+txt.size());
		check = 2+txt.size();
		Send(in);
		check += in;
		check %= 256;
		for(i=0;i<txt.size();i++) {
			Send(txt_char[i]);
			check += txt_char[i];
			check %= 256;
		}
		Send(check);
	} else {
		std::cout << "chaine trop longue max 15" << std::endl;
	}
	m_pSpi->unlock();

}
void HMI::SetPos(int in, int pos) {
	m_pSpi->lock();
	if(m_pSpi->getSlaveId() != m_id){	
		m_pSpi->setSlave(m_id);		//change Chip select
	}
	in += 6;
	int lsb = pos%256, msb = (int)(pos/256);
	Send(4);
	Send(in);
	Send(msb);
	Send(lsb);
	Send(4+in+msb+lsb);
	m_pSpi->unlock();
}
void HMI::SetTxt(int in, std::string txt) {
	m_pSpi->lock();
	if(m_pSpi->getSlaveId() != m_id){	
		m_pSpi->setSlave(m_id);		//change Chip select
	}
	EraseScreen(in);
	delay(ERASE_TIME);
	unsigned int i; 
	int check = 0;
	in += 7;
	char txt_char[txt.size()+1];
	strcpy(txt_char, txt.c_str());
	if(txt.size() < 197) {
		Send(2+txt.size());
		check = 2+txt.size();
		Send(in);
		check += in;
		check %= 256;
		for(i=0;i<txt.size();i++) {
			Send(txt_char[i]);
			check += txt_char[i];
			check %= 256;
		}
		Send(check);
	} else {
		std::cout << "erreur chaine trop longue max 196" << std::endl;
	}
	m_pSpi->unlock();
}
void HMI::SetTxtFull(std::string txt_in) {
	m_pSpi->lock();
	if(m_pSpi->getSlaveId() != m_id){	
		m_pSpi->setSlave(m_id);		//change Chip select
	}
	if(txt_in.size() < 197) {
		SetTxt(1, txt_in.substr(0,txt_in.size()));
		delay(ERASE_TIME);
		EraseScreen(2);
		delay(ERASE_TIME);
		EraseScreen(3);
		delay(ERASE_TIME);
		EraseScreen(4);
	} else if(txt_in.size() < 392) {
		SetTxt(1, txt_in.substr(0,196));
		delay(ERASE_TIME);
		SetTxt(2, txt_in.substr(196,(txt_in.size()-196)));
		delay(ERASE_TIME);
		EraseScreen(3);
		delay(ERASE_TIME);
		EraseScreen(4);
	} else if(txt_in.size() < 588) {
		SetTxt(1, txt_in.substr(0,196));
		delay(ERASE_TIME);
		SetTxt(2, txt_in.substr(196,196));
		delay(ERASE_TIME);
		SetTxt(3, txt_in.substr(392,txt_in.size()-392));
		delay(ERASE_TIME);
		EraseScreen(4);
	} else if(txt_in.size() < 784) {
		SetTxt(1, txt_in.substr(0,196));
		delay(ERASE_TIME);
		SetTxt(2, txt_in.substr(196,196));
		delay(ERASE_TIME);
		SetTxt(2, txt_in.substr(392,196));
		delay(ERASE_TIME);
		SetTxt(3, txt_in.substr(588,txt_in.size()-588));
	} else {
		std::cout << "erreur chaine trop longue";
	}
	m_pSpi->unlock();
}
void HMI::EraseScreen(int in_era) {
	m_pSpi->lock();
	if(m_pSpi->getSlaveId() != m_id){	
		m_pSpi->setSlave(m_id);		//change Chip select
	}
	int check_era = 198, i_era;
	Send(198);
	in_era += 7;
	Send(in_era);
	check_era += in_era;
	check_era %= 256;
	for(i_era=0;i_era<196;i_era++) {
		Send('\0');
	}
	Send(check_era);
	m_pSpi->unlock();
}
void HMI::flush(uint16_t nb){
	m_pSpi->lock();
	if(m_pSpi->getSlaveId() != m_id){
		m_pSpi->setSlave(m_id);		//change Chip select
	}
	for(uint16_t i = 0; i < nb; i++){
		Send(0);
	}
	m_pSpi->unlock();
}
void HMI::resetPic(void){
	m_pSpi->lock();
	if(m_pSpi->getSlaveId() != m_id){
		m_pSpi->setSlave(m_id);		//change Chip select
	}
	Send(2);
	Send(14);
	Send(16);
	m_pSpi->unlock();
}