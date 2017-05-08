#ifndef DAQ_H
#define DAQ_H

#include <QThread>
#include "wiringPi.h"


#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
//#include <curses.h>
#include <endian.h>


struct DASHdata {
    int rpm[1024];
    int etc[1024];
    int bat[1024];

};

class DAQ : public QThread
{
    Q_OBJECT
public:
    explicit DAQ(QObject *parent = 0);
    void run();
    bool StopThread;
    void storageInit();
    void saveData ();
    bool fileExists(QString path);

    void unknown_frame(int id);
    void process_one(struct can_frame *frm);
    int net_init(char *ifname);
    void receive_one(void);


    int k;
signals:
    void onInterrupt (int, int, int, int, int);
public slots:
    
};

#endif // DAQ_H
