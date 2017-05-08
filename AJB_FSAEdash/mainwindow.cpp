#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "MAIN2.h"


///////////////////////////////////////////////////
//Variables//
///////////////////////////////////////////////////

int engineTemp=500; //temperature of coolant
int speed=3;    //speed
int RPM=6000;  //RPM
int batt=12;    //battery volatge
int gas=79;     //percent gas in tank
int oilTemp=120; //oil temperature
int gear=1;     //gear the car is in
int dial=1;     //screen preference dial
double lTime=1.25;   //lap time



double testc=1;



///////////////////////////////////////////////////////////

////////////////////////////////
//variables from austin's code//
////////////////////////////////

//Thermal Array//
int FL=0;
int FR=0;
int BR=0;
int BL=0;

//IMU//
int xAccel=0;
int yAccel=0;
int zAccel=0;
int xRot=0;
int yRot=0;
int zRot=0;
int temp=0;

//ADC//


int steerAngle=0;
int airSpeed=0;
int brakePressF=0;
int brakePressB=0;
int shockDispFL=0;
int shockDispFR=0;
int shockDispBR=0;
int shockDispBL=0;


//RTC//

int startTime=0;
int elapsedTime=0;
int timeStamp=0;
int interval=0;

///intitializing test variabelsa

float value0=0;
float value1=0;
float value2=0;
float value3=0;



//ECU//
//extern int engineTemp;
//extern int waterTemp;
//extern int rpm;
//extern int throttlePos;
//extern int oilTemp;
//extern int oilPress;
//extern int lambda;
//////////////////////////////////////////////

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowFlags(Qt::FramelessWindowHint);


   // timer= new QTimer(this);




    ui->MessageM->setTextColor(Qt::red);
    ui->MessageM->setFontPointSize(26);

    ui->MessageM->setTextColor(Qt::red);
    ui->MessageM->setFontPointSize(26);

    ui->MessageS->setTextColor(Qt::white);
    ui->MessageS->setFontPointSize(26);

<<<<<<< HEAD
    ui->tabWidget->setCurrentIndex(4);

    //connect(timer, SIGNAL(timeout()), this, SLOT(update()));
=======
  //  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
>>>>>>> 94dce0f3697953a2780148a9a4cb099c53f40500

  //  timer->start(20);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::update(void)
{
<<<<<<< HEAD
    
    //*****************************************************************
    // SETUP, PRE-SAMPLING
    //*****************************************************************

=======
/*
>>>>>>> 4ad490291898b4522e187fbbd7c2cc97eb0ee879
    int val;
    i2cBus i2c;
    ADC adc16;
<<<<<<< HEAD

    val = adc16.getValue(5);
    value0= (float)val*6.144/32767.0;
    val = adc16.getValue(6);
    value1= (float)val*6.144/32767.0;
    val = adc16.getValue(7);
    value2= (float)val*6.144/32767.0;
    val = adc16.getValue(8);
    value3= (float)val*6.144/32767.0;
*/
    //following will set the screen to display data//

    ///Dynamic 1//
    ui->enginetemp->display(engineTemp);
    ui->RPMnumber->display(RPM);
    ui->Speed->display(speed);
    ui->gearnumber->display(gear);
    ui->LapTime->display(lTime);
    ui->gas->setValue(gas);

    //Dynamic 2///
    ui->progressBar->setValue(RPM);
    ui->enginetemp_2->display(engineTemp);
    ui->Speed_2->display(speed);
    ui->gearnumber_2->display(gear);
    ui->LapTime_4->display(lTime);
    ui->gas_2->setValue(gas);

    //static//
    ui->BattS->display(batt);
    ui->CoolantTempS->display(engineTemp);
    ui->EngineSpeedS->display(RPM);
    ui->GasS->display(gas);
    ui->OilTempS->display(oilTemp);


    ///test screen///

    ui->ch1->display(value0);
    ui->ch2->display(value1);
    ui->ch3->display(value2);
    ui->ch4->display(value3);




   // Message Stuff

    if (engineTemp>240)
    {

        ui->MessageS->setPlainText("Engine is too hot");
        ui->MessageM->setPlainText("Engine is too hot");
        ui->MessageM_2->setPlainText("Engine is too hot");

    }
    else
        if(RPM>15000)
        {

            ui->MessageS->setPlainText("Too Accelerated: Let off Throttle");
            ui->MessageM->setPlainText("Too Accelerated: Let off Throttle");
            ui->MessageM_2->setPlainText("Too Accelerated: Let off Throttle");

=======
    RTC rtc;
    
    int i,value;
    int fd;
    int res_ds3231_control;
    
    //------------------initialization of counters------------------
    unsigned char accelCount = 0;
    unsigned char gyroCount = 0;
    unsigned char tempCount = 0;
    unsigned char gpsCount = 0;
    unsigned char brakePressCount = 0;
    unsigned char shockCount = 0;
    unsigned char tharCount = 0;
    unsigned char steerAngleCount = 0;
    unsigned char airSpeedCount = 0;
    //---------------initialization of main counter----------------
    unsigned char count = 0;
    
    //----------------initialization of isr freq------------------
    unsigned char isrFreq = 512;
    
    ///////////////////////////////////////////
    //isrFreq is actually 512??? should adjust other frequencies to a 2^xxxxx value, but completely divisible??
    ///////////////////////////////////////////
    //--------initialization sensor sampling frequencies----------
    unsigned char accelFreq = 128;
    unsigned char gyroFreq = 128;
    unsigned char tempFreq = 1;
    unsigned char gpsFreq = 8;
    unsigned char brakePressFreq = 8;
    unsigned char shockFreq = 512;
    unsigned char tharFreq = 8;
    unsigned char steerAngleFreq = 8;
    unsigned char airSpeedFreq = 8;
    
    //----------calculations of counter sampling values---------------
    //----------------------(DO NOT CHANGE)---------------------------
    accelFreq = isrFreq/accelFreq;
    gyroFreq = isrFreq/gyroFreq;
    tempFreq = isrFreq/tempFreq;
    gpsFreq = isrFreq/gpsFreq;
    brakePressFreq = isrFreq/brakePressFreq;
    shockFreq = isrFreq/shockFreq;
    tharFreq = isrFreq/tharFreq;
    steerAngleFreq = isrFreq/steerAngleFreq;
    airSpeedFreq = isrFreq/airSpeedFreq;
    
    
    
    //wiringPi initialization
	if (wiringPiSetup () < 0)	{
		fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno)) ;

	}
	if (wiringPiISR (BUTTON_PIN, INT_EDGE_RISING, &isr) < 0)	{ //rising edge triggered
		fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno)) ;
	}
    
    rtc.init(DS3231_1Hz); // initialize interrupt to 1 hz
    
    
    
    //*****************************************************************
    // INTERRUPT SERVICE ROUTINE
    //*****************************************************************
    
    
    FILE *stream;
    int i,k,val;
    int fd;
    volatile int count = 0;
    int countChange = 0;
    unsigned char second;
    float value;
    i2cBus i2c;
    RTC rtc;
    ADC adc;
    int countBuffer[1024];
    float adcBuffer[1024];
    unsigned char secondBuffer[1024];
    
    void interrupt () {
        
        
        count ++;
     
        if (accelCount >= accelFreq) {
            //get accel data
            accelCount = 0;
     
        }
        else {accelCount++;}
        
        if (gyroCount >= gyroFreq) {
            //get gyro data
            gyroCount = 0;
            
        }
        else {gyroCount++;}
        
        if (tempCount >= tempFreq) {
            //get temp data
            
            
            tempCount = 0;
            
        }
        else {tempcount++;}
        
        if (gpsCount >= gpsFreq) {
            //get gps data
            gpsCount = 0;
            
        }
        else {gpsCount++;}
        
        if (brakePressCount >= brakePressFreq) {
            //get brake pressure data
            brakePressCount = 0;
            
        }
        else {brakePressCount++;}
        
        if (shockCount >= shockFreq) {
            //get shock pot data
            shockCount = 0;
            
        }
        else {shockCount++;}
        
        if (tharCount >= tharFreq) {
            //get thar data
            tharCount = 0;
            
        }
        else {tharCount++;}
        
        if (steerAngleCount >= steerAngleFreq) {
            //get steering angle data
            steerAngleCount = 0;
            
        }
        else {steerAngleCount++;}
        
        if (airSpeedCount >= airSpeedFreq) {
            //get airspeed data
            airSpeedCount = 0;
            
        }
        else {airspeedCount++;}
        
        
        //write conditional for writing data to SD card
        
        unsigned char readBuf[2];
        readBuf[0]= 0;
        readBuf[1]= 0;
        unsigned char writeBuf[1];
        writeBuf[0] = DS3231_SECOND;
        
        i2c.setAddress(DS3231_SLAVE_ADDR);
        i2c.send(writeBuf,1);
        i2c.receive(readBuf,1)&0x7F;
        second = readBuf[0];
        
        
        val = adc.getValue(5);
        value= (float)val*6.144/32767.0;
        //float value = 0;
        stream = fopen("speedtest.txt","a");
        
        //adcBuffer[count-1] = value;
        //secondBuffer [count-1] = second;
        //countBuffer[count-1] = count;
        //printf("%x  %d  %f\n", countBuffer[count-1], countBuffer[count-1], adcBuffer[count-1]);
        
        fprintf(stream, "%x   %d   %f\n", second , count, value);
        fclose (stream);
        
        //printf("interrupt\n");
        
    }
    
    
    
    
    //*****************************************************************
    // ENTER MAIN LOOP
    //*****************************************************************


    while (1) {
        
        
        

        //following will set the screen to display data//

        ///Screen 1//
        ui->enginetemp->display(engineTemp);
        ui->RPMnumber->display(RPM);
        ui->Speed->display(speed);
        ui->gearnumber->display(gear);
        ui->LapTime->display(lTime);

        //Screen 2///
        ui->progressBar->setValue(RPM);

        //stopped screen//
        ui->BattS->display(batt);
        ui->CoolantTempS->display(engineTemp);
        ui->EngineSpeedS->display(RPM);
        ui->GasS->display(gas);
        ui->OilTempS->display(oilTemp);


        ///test screen///

        ui->ch1->display(value0);
        ui->ch2->display(value1);
        ui->ch3->display(value2);
        ui->ch4->display(value3);


        ui->ch1->hide();
        ui->ch1l->hide();
        ui->ch2->hide();
        ui->ch2l->hide();
        ui->ch3->hide();
        ui->ch3l->hide();
        ui->ch4->hide();
        ui->ch4l->hide();


       // Message Stuff

        if (engineTemp>240)
        {

            ui->MessageS->setPlainText("Engine is too hot");
            ui->MessageM->setPlainText("Engine is too hot");
>>>>>>> 94dce0f3697953a2780148a9a4cb099c53f40500
        }
        else
            if(RPM>15000)
            {

<<<<<<< HEAD
                ui->MessageS->setPlainText("Oil Too Hot");
                ui->MessageM->setPlainText("Oil Too Hot");
                ui->MessageM_2->setPlainText("Oil Too Hot");
=======
                ui->MessageS->setPlainText("Too Accelerated: Let off Throttle");
                ui->MessageM->setPlainText("Too Accelerated: Let off Throttle");
>>>>>>> 94dce0f3697953a2780148a9a4cb099c53f40500

            }
            else
                if(oilTemp>250)
                {

<<<<<<< HEAD
                    ui->MessageS->setPlainText("Check Battery");
                    ui->MessageM->setPlainText("Check Battery");
                    ui->MessageM_2->setPlainText("Check Battery");
=======
                    ui->MessageS->setPlainText("Oil Too Hot");
                    ui->MessageM->setPlainText("Oil Too Hot");
>>>>>>> 94dce0f3697953a2780148a9a4cb099c53f40500

                }
                else
                    if(batt>14 || batt<9)
                    {

<<<<<<< HEAD
                        ui->MessageS->setPlainText("Put More Gas!!");
                        ui->MessageM->setPlainText("Put More Gas!!");
                        ui->MessageM_2->setPlainText("Put More Gas!!");
=======
                        ui->MessageS->setPlainText("Check Battery");
                        ui->MessageM->setPlainText("Check Battery");
>>>>>>> 94dce0f3697953a2780148a9a4cb099c53f40500

                    }
                    else
                        if(gas<10)
                        {

                            ui->MessageS->setPlainText("Put More Gas!!");
                            ui->MessageM->setPlainText("Put More Gas!!");

<<<<<<< HEAD
                    }

    if(speed>2)
    {
        if(dial==1){

            ui->tabWidget->setCurrentIndex(1);

        }
        if(dial==2){
            ui->tabWidget->setCurrentIndex(2);
=======
                        }
                        else
                            {

                                ui->MessageS->setPlainText("Good To Go!!!");


                        }

        if(speed>2)
        {
            //hide stopped screen//
            ui->BattS->hide();
            ui->BlabelS->hide();
            ui->CoolantTempS->hide();
            ui->EngineSpeedS->hide();
            ui->GasS->hide();
            ui->CTLS->hide();
            ui->GLS->hide();
            ui->lineS->hide();
            ui->LSpeedS->hide();
            ui->MessageS->hide();
            ui->OilTempS->hide();
            ui->OTLS->hide();



            //show rest of screen//
            ui->enginetemp->show();
            ui->gearnumber->show();
            ui->label->show();
            ui->label_2->show();
            ui->label_3->show();
            ui->label_4->show();
            ui->label_5->show();
            //ui->label_6->show();
            ui->LapTime->show();
            ui->MessageM->show();
            ui->Speed->show();



            if(dial==1){
            ui->progressBar->hide();
            ui->RPMnumber->show();
>>>>>>> 94dce0f3697953a2780148a9a4cb099c53f40500

            }
            if(dial==2){
                ui->progressBar->show();
                ui->RPMnumber->hide();

                //set rpm color//
                if(RPM>7000){
                    ui->progressBar->setStyleSheet("selection-background-color: rgb(0, 255, 0);");
                }
                if(RPM>12000){
                    ui->progressBar->setStyleSheet("selection-background-color: rgb(255, 0, 0);");
                }
            }

        }

<<<<<<< HEAD
    if(speed<=2){

        //show the stopped screen//
        ui->tabWidget->setCurrentIndex(0);
    }

    //test screen stuff//

    if (testc==1){
        ui->tabWidget->setCurrentIndex(3);
    }
=======
        if(speed<=2){

            //show the stopped screen//
            ui->BattS->show();
            ui->BlabelS->show();
            ui->CoolantTempS->show();
            ui->EngineSpeedS->show();
            ui->GasS->show();
            ui->CTLS->show();
            ui->GLS->show();
            ui->lineS->show();
            ui->LSpeedS->show();
            ui->MessageS->show();
            ui->OilTempS->show();
            ui->OTLS->show();


            //hide the previous screens///
            ui->enginetemp->hide();
            ui->gearnumber->hide();
            ui->label->hide();
            ui->label_2->hide();
            ui->label_3->hide();
            ui->label_4->hide();
            ui->label_5->hide();
            //ui->label_6->hide();
            ui->LapTime->hide();
            ui->MessageM->hide();
            ui->Speed->hide();
            ui->RPMnumber->hide();
            ui->progressBar->hide();
        }

        //test screen stuff//

        if (testc==1){
            ui->enginetemp->hide();
            ui->gearnumber->hide();
            ui->label->hide();
            ui->label_2->hide();
            ui->label_3->hide();
            ui->label_4->hide();
            ui->label_5->hide();
            //ui->label_6->hide();
            ui->LapTime->hide();
            ui->MessageM->hide();
            ui->Speed->hide();
            ui->RPMnumber->hide();
            ui->progressBar->hide();
            ui->BattS->hide();
            ui->BlabelS->hide();
            ui->CoolantTempS->hide();
            ui->EngineSpeedS->hide();
            ui->GasS->hide();
            ui->CTLS->hide();
            ui->GLS->hide();
            ui->lineS->hide();
            ui->LSpeedS->hide();
            ui->MessageS->hide();
            ui->OilTempS->hide();
            ui->OTLS->hide();

            ui->ch1->show();
            ui->ch1l->show();
            ui->ch2->show();
            ui->ch2l->show();
            ui->ch3->show();
            ui->ch3l->show();
            ui->ch4->show();
            ui->ch4l->show();
        }
>>>>>>> 94dce0f3697953a2780148a9a4cb099c53f40500


            
    }
}



