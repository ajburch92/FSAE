#include "GPS_V3.h"
#include <stdexcept>
#include <string>
int previous_speed = 0;
std::string previous_GPSRMC = "$GPRMC,2255.40,2A,3037,2347,N,,2620.45,2,W,0.1,228.421,20416,1,2";
//Open Port & Set Port
int GPS::GPSInit(void){
	nFd = open(GPS_DEVICE, O_RDWR|O_NOCTTY|O_NDELAY);
	if(-1 == nFd){
		perror("Open Serial Port Error!\n");
		return -1;
	}
	if( (fcntl(nFd, F_SETFL, 0)) < 0 ){
		perror("Fcntl F_SETFL Error!\n");
		return -1;
	}
	if(tcgetattr(nFd, &stOld) != 0){
		perror("tcgetattr error!\n");
		return -1;
	}
	stNew = stOld;
	cfmakeraw(&stNew);//raw mode
	//set speed
	cfsetispeed(&stNew, GPS_BAUDRATE);//from header file, baudrate  = 9600
	cfsetospeed(&stNew, GPS_BAUDRATE);
	//set databits
	stNew.c_cflag |= (CLOCAL|CREAD);
	stNew.c_cflag &= ~CSIZE;
	stNew.c_cflag |= CS8;
	//set parity
	stNew.c_cflag &= ~PARENB;
	stNew.c_iflag &= ~INPCK;
	//set stopbits
	stNew.c_cflag &= ~CSTOPB;
	stNew.c_cc[VTIME]=0;    //?????????????
	stNew.c_cc[VMIN]=1; //??????????????,??????n*100ms
	//????VTIME=0,???????read()????????
	tcflush(nFd,TCIFLUSH);  //??????????/????????
	if( tcsetattr(nFd,TCSANOW,&stNew) != 0 ){
		perror("tcsetattr Error!\n");
		return -1;
	}
    
	nRetw = write(nFd, PMTK_SET_BAUD_9600, std::char_traits<char>::length(PMTK_SET_BAUD_9600));
	if (debug == 1){
		printf("nRetw = %d\n", nRetw);
	}
	nRetw = write(nFd, PMTK_API_SET_SBAS_ENABLED, std::char_traits<char>::length(PMTK_API_SET_SBAS_ENABLED));
	if (debug == 1){
		printf("nRetw = %d\n", nRetw);
	}
	nRetw = write(nFd, PMTK_SET_NMEA_OUTPUT_RMCONLY, std::char_traits<char>::length(PMTK_SET_NMEA_OUTPUT_RMCONLY));
	if (debug == 1){
		printf("nRetw = %d\n", nRetw);
	}
	nRetw = write(nFd, PMTK_SET_NMEA_UPDATE_10HZ, std::char_traits<char>::length(PMTK_SET_NMEA_UPDATE_10HZ));
	if (debug == 1){
		printf("nRetw = %d\n", nRetw);
	}
	if(-1 == nRetw){
		perror("Write Data Error!\n");
		return -1;
	}
	bzero(buf, GPS_BUFFER_SIZE);
	return nFd;
}

std::string GPS::printGPSRMC(void){
	printf("get sentence");
	bzero(buf, GPS_BUFFER_SIZE);
	storage = "";


	nRet = read(nFd, buf, GPS_BUFFER_SIZE);
	if(-1 == nRet){
		perror("Read Data Error!\n");
	}
	if(0 < nRet){
		if (storage == "" && buf[0] == '$' && buf[1] == 'G'){
			storage.append(buf);
			for (int j = 0; j < 7; j++){
				nRet = read(nFd, buf, GPS_BUFFER_SIZE);
				if (buf[0] != '$'){
					storage.append(buf);
				}
				else j = 7;
			}
		previous_GPSRMC = storage;
		}
		else if(storage ==""){
			printf("trying again... ");
			//usleep(1000000);
			//GPS::printGPSRMC();
			return previous_GPSRMC;
		}
	}
    
	return storage;
	//close(nFd);
}
int GPS::printspeed (std::string GPSRMC_sentence){
	//inspired by http://stackoverflow.com/questions/1894886/parsing-a-comma-delimited-stdstring
	std::istringstream ss(GPSRMC_sentence);
	std::string token;
	std::string::size_type sz;     // alias of size_t
	int speed_in_mph;
	double speed_in_knot;
	int i;
	for (int i = 0; i <10; i++){
		std::getline(ss, token, ',');
		if (token == "W"){
			std::getline(ss, token, ',');
			break;
		}

	}	
	try{ 
		speed_in_knot = std::stod (token,&sz);
		speed_in_mph = ceil(speed_in_knot * 0.868976242);
	}
	catch(const std::invalid_argument&) {
	
	speed_in_mph = previous_speed;
	}

	previous_speed = speed_in_mph;    
	
	return speed_in_mph;
}

int main(int argc, char **argv){
	GPS gps;
	gps.GPSInit();
	printf("gps initialized");

	while(1){
		std::string output = gps.printGPSRMC();
		//int speed = gps.printspeed(output);
		std::cout<<"\n"<<output<<"\n";
		int speed = gps.printspeed(output);
		std::cout<<speed<<"\n";
		usleep(1000000);
	}
	return 0;
}
