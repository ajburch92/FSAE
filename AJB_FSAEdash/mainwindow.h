#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


public slots:
    void update(void);
    
private:
    Ui::MainWindow *ui;


    QTimer *timer;
};

extern int engineTemp;
extern int speed;
extern int RPM;
extern int batt;
extern int gas;
extern int oilTemp;
extern int gear;
extern int dial;
extern double lTime;

///////////////////////////////////////////////////////////

////////////////////////////////
//variables from austin's code//
////////////////////////////////

//Thermal Array//
extern int FL;
extern int FR;
extern int BR;
extern int BL;

//IMU//
extern int xAccel;
extern int yAccel;
extern int zAccel;
extern int xRot;
extern int yRot;
extern int zRot;
extern int temp;

//ADC//


extern int steerAngle;
extern int airSpeed;
extern int brakePressF;
extern int brakePressB;
extern int shockDispFL;
extern int shockDispFR;
extern int shockDispBR;
extern int shockDispBL;


//RTC//

extern int startTime;
extern int elapsedTime;
extern int timeStamp;
extern int interval;


//ECU//
//extern int engineTemp;
//extern int waterTemp;
//extern int rpm;
//extern int throttlePos;
//extern int oilTemp;
//extern int oilPress;
//extern int lambda;



//test variabels
extern double testc;
extern int i;






#endif // MAINWINDOW_H
