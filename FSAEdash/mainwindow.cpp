#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <wiringPi.h>

//go to th`e projects tab on the left//
//uncheck shadow build//
//run program//
//but program will not run//
//re check the shadow build shit//
//run program///


int ETC=0; //temperature of coolant
int Speed=0;    //speed
int RPM=0;  //RPM
int Batt=0;    //battery volatge
int TPS=0;
//int gas=79;     //percent gas in tank
//int oilTemp=120; //oil temperature
//int gear=1;     //gear the car is in
int dial=1;     //screen preference dial
//double lTime=0.00;   //lap time



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QThread::currentThread()->setPriority(QThread::LowPriority);

    DAQthread = new DAQ(this);
    connect (DAQthread,SIGNAL(onInterrupt(int, int, int, int, int)),this,SLOT(update(int, int, int ,int, int)));

    QMainWindow::showFullScreen();
    this->setWindowFlags(Qt::FramelessWindowHint);

    DAQthread ->StopThread = false;
    DAQthread ->start(QThread::HighestPriority);

    ui->MessageD1->setTextColor(Qt::red);
    ui->MessageD1->setFontPointSize(20);
    ui->MessageD1->setTextColor(Qt::red);
    ui->MessageD1->setFontPointSize(20);
    ui->MessageD2->setTextColor(Qt::red);
    ui->MessageD2->setFontPointSize(20);
    ui->MessageD2->setTextColor(Qt::red);
    ui->MessageD2->setFontPointSize(20);
    ui->MessageS->setTextColor(Qt::white);
    ui->MessageS->setFontPointSize(20);
    ui->tabWidget->setCurrentIndex(1);

    setupPins();
    //pinMode(5,OUTPUT);
    //digitalWrite(5,HIGH);
}
MainWindow::~MainWindow()
{
    printf("ending DAQthread\n");
    printf("closing MainWindow\n");
    DAQthread-> StopThread = true;
    delete ui;
}



void MainWindow::update(int speed, int rpm, int etc, int bat, int tps)
{
    RPM = rpm;
    ETC = etc;
    Speed = speed;
    Batt = bat;
    TPS = tps;
    // screen select:
    dial=digitalRead(4); //screen select wiring pi

    //if(Speed >= 1)
    //{
    if(dial<1){
        ///Dynamic 1//
        ui->ETCD1->display(ETC);
        ui->RPMD1->display(RPM);
        ui->speedD1->display(Speed);
        ui->BATprogD1->setValue(Batt);
        ui->tabWidget->setCurrentIndex(1);

    }
    else {
        //Dynamic 2///
        ui->RPMprogD2->setValue(RPM);
        ui->ETCD2->display(ETC);
        ui->speedD2->display(Speed);
        ui->BATprogD2->setValue(Batt);
        ui->tabWidget->setCurrentIndex(2);

        //set rpm color//
        if (RPM<= 7000) {
            ui->RPMprogD2->setStyleSheet("selection-background-color: rgb(0, 255, 0);");
        }
        if(RPM>7000){
            ui->RPMprogD2->setStyleSheet("selection-background-color: rgb(255, 255, 0);");
        }
        if(RPM>10000){
            ui->RPMprogD2->setStyleSheet("selection-background-color: rgb(255, 0, 0);");
        }
        //}

    }

    //  else {
    //show the stopped screen//
    //                //static//
    //                ui->BattS->display(batt);
    //                ui->CoolantTempS->display(engineTemp);
    //                ui->EngineSpeedS->display(RPM);
    //                ui->GasS->display(gas);
    //                ui->OilTempS->display(oilTemp);
    //      ui->tabWidget->setCurrentIndex(0);
    //  }


    if (Batt>14 || Batt<9)
    {

        ui->MessageS->setPlainText("Warning : Check Battery");
        ui->MessageD1->setPlainText("Warning : Check Battery");
        ui->MessageD2->setPlainText("Warning : Check Battery");
    }

    else if(ETC>=235)
    {

        ui->MessageS->setPlainText("Warning : Engine Coolant Temperature");
        ui->MessageD1->setPlainText("Warning : Engine Coolant Temperature");
        ui->MessageD2->setPlainText("Warning : Engine Coolant Temperature");

    }

    else

    {

        ui->MessageS->setPlainText("---TAR16---");
        ui->MessageD1->setPlainText("---TAR16---");
        ui->MessageD2->setPlainText("---TAR16---");
    }

}



void MainWindow::setupPins()
{
//    wiringPiSetup ();
    pinMode(4, INPUT);
    printf("wiringPi pins initialized\n");


}







