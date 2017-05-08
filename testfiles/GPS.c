#include "GPS.h"
//Its a good idea to have a debug function
FILE *stream;
int nFd = 0;
struct termios stNew;
struct termios stOld;
//Open Port & Set Port
int SerialInit(){
	nFd = open(DEVICE, O_RDWR|O_NOCTTY|O_NDELAY);
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
	cfsetispeed(&stNew, BAUDRATE);//from header file, baudrate  = 9600
	cfsetospeed(&stNew, BAUDRATE);
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
	return nFd;
}
int main(int argc, char **argv){
	int nRet = 0;
	int nRetw = 0;
	char buf[SIZE];
	int recnum = 0;
	if( SerialInit() == -1 ){
		perror("SerialInit Error!\n");
		return -1;
	}
	bzero(buf, SIZE);
	nRetw = write(nFd, PMTK_SET_BAUD_9600, strlen(PMTK_SET_BAUD_9600));
	printf("nRetw = %d\n", nRetw);
	nRetw = write(nFd, PMTK_SET_NMEA_OUTPUT_RMCONLY, strlen(PMTK_SET_NMEA_OUTPUT_RMCONLY));
	printf("nRetw = %d\n", nRetw);
	nRetw = write(nFd, PMTK_SET_NMEA_UPDATE_10HZ, strlen(PMTK_SET_NMEA_UPDATE_10HZ));
	printf("nRetw = %d\n", nRetw);
	nRetw = write(nFd, PMTK_API_SET_FIX_CTL_5HZ, strlen(PMTK_API_SET_FIX_CTL_5HZ));
	printf("nRetw = %d\n", nRetw);
	if(-1 == nRetw){
		perror("Write Data Error!\n");
	}
	while(1){
		
		nRet = read(nFd, buf, SIZE);
		if(-1 == nRet){
			perror("Read Data Error!\n");
			break;
		}
		if(0 < nRet){
			//buf[nRet] = 0;
			//stream = fopen("FSAEdata.txt","a");
			//fprintf(stream, "%s", buf );
			//?????HOW TO FILTER NEMA INFO????? 
			//fclose (stream);
			printf("%s", buf);
			//printf("Recv Data: %s\n", buf);
			//printf("%d\n", recnum);
			//recnum++;
		}
	}
	close(nFd);
	return 0;
}
