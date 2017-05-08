#include <stdio.h>
#include <wiringPi.h>
#include <sr595.h>


class Dash_LED{
	public:
		Dash_LED();
		int single_LED_on(int LED_num);
		int single_LED_off(int LED_num);
		int sequential_light(int delaytime);
		int all_LED_control(int LED_byte);
	private:

};

	
Dash_LED::Dash_LED(){
	//Use wiringPi pins 0, 1 & 2 for data, clock and latch
	printf("settingup");
	//sr595Setup (100, 8, 0, 1, 2);
	sr595Setup (100, 8, 3, 4, 1);
	printf("stsetup");
}

int Dash_LED::single_LED_on(int LED_num){
	
	digitalWrite(LED_num+100, 1);
	return 0;
}

int Dash_LED::single_LED_off(int LED_num){
	
	digitalWrite(LED_num+100, 0);
	return 0;
}
	
int Dash_LED::sequential_light(int delaytime){
	
	for (int i = 0; i < 8; i++){
		single_LED_on(i);
		delay(delaytime);
		single_LED_off(i);
	}
		/* single_LED_on(7);
		delay(2000);
		single_LED_off(7); */
	for (int j = 7; j >= 0; j--){

		single_LED_on(j);
		delay(delaytime);
		single_LED_off(j);
	}

	return 0;
}

int Dash_LED::all_LED_control(int LED_byte){ //input can be either binary, decimal, or hexadecimal
	for( int i = 7; i <= 0; i++){
		if (LED_byte /(2^i) != 0){//current digit  != 0
			single_LED_on(i);
		}
		else single_LED_off(i);//current digit = 0
		//LED_byte = LED_byte % (2^i); // not necessary. 
	}
	return 0;
}

	
int main (void){
	printf("starting");
 	wiringPiSetup() ;
	printf("creating object");
	Dash_LED LED;
	printf("setup");
	LED.sequential_light(500);//start up! 
	//LED.all_LED_control(0b11111111);
	//printf("done");
	LED.all_LED_control(0b10000010);//turn on one led at both ends on the dash (not including the push button led)
	LED.all_LED_control(0b11000110);//turn on two leds at both ends (not including the push button led)
	printf("done");
	

  return 0 ;
}
