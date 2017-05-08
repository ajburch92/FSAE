#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "daq.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    DAQ *DAQthread;

public slots:
   void update(int, int, int, int, int);
   void setupPins();

private:
    Ui::MainWindow *ui;

};



#endif // MAINWINDOW_H
