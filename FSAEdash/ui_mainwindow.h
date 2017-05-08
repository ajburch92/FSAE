/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QProgressBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTextBrowser>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTabWidget *tabWidget;
    QWidget *Static;
    QLCDNumber *GasS;
    QTextBrowser *BlabelS;
    QLCDNumber *BatS;
    QTextBrowser *LSpeedS;
    QLCDNumber *CoolantTempS;
    QLCDNumber *RPMS;
    QLCDNumber *OilTempS;
    QTextBrowser *MessageS;
    QWidget *Dynamic1;
    QLCDNumber *speedD1;
    QLCDNumber *ETCD1;
    QTextBrowser *MessageD1;
    QLCDNumber *RPMD1;
    QProgressBar *BATprogD1;
    QLabel *label_3;
    QLCDNumber *EXTRAD1;
    QLabel *label_10;
    QWidget *Dynamic2;
    QProgressBar *RPMprogD2;
    QProgressBar *EXTRAprogD2;
    QLCDNumber *EXTRAD2;
    QLabel *label_8;
    QTextBrowser *MessageD2;
    QLabel *label_4;
    QLCDNumber *speedD2;
    QLCDNumber *ETCD2;
    QProgressBar *BATprogD2;
    QWidget *Test;
    QLabel *ch1l;
    QLabel *ch2l;
    QLCDNumber *ch2;
    QLCDNumber *ch1;
    QLabel *ch4l;
    QLCDNumber *ch4;
    QLabel *ch3l;
    QLCDNumber *ch3;
    QWidget *tab;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(720, 481);
        MainWindow->setMaximumSize(QSize(800, 700));
        MainWindow->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 0, 0);"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 0, 721, 481));
        tabWidget->setMaximumSize(QSize(16777215, 16777215));
        tabWidget->setSizeIncrement(QSize(12, 0));
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(27, 31, 34, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush1);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        tabWidget->setPalette(palette);
        QFont font;
        font.setFamily(QString::fromUtf8("Roboto"));
        font.setBold(true);
        font.setWeight(75);
        tabWidget->setFont(font);
        tabWidget->setCursor(QCursor(Qt::BusyCursor));
        tabWidget->setLayoutDirection(Qt::LeftToRight);
        tabWidget->setAutoFillBackground(false);
        tabWidget->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(27,31,34);"));
        tabWidget->setTabPosition(QTabWidget::South);
        tabWidget->setTabShape(QTabWidget::Rounded);
        tabWidget->setIconSize(QSize(14, 16));
        tabWidget->setElideMode(Qt::ElideNone);
        tabWidget->setDocumentMode(true);
        tabWidget->setTabsClosable(false);
        tabWidget->setMovable(false);
        Static = new QWidget();
        Static->setObjectName(QString::fromUtf8("Static"));
        GasS = new QLCDNumber(Static);
        GasS->setObjectName(QString::fromUtf8("GasS"));
        GasS->setGeometry(QRect(40, 210, 101, 91));
        GasS->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);"));
        GasS->setLineWidth(-1);
        GasS->setMidLineWidth(1);
        GasS->setNumDigits(2);
        GasS->setDigitCount(2);
        GasS->setProperty("value", QVariant(0));
        BlabelS = new QTextBrowser(Static);
        BlabelS->setObjectName(QString::fromUtf8("BlabelS"));
        BlabelS->setGeometry(QRect(170, -50, 141, 41));
        BatS = new QLCDNumber(Static);
        BatS->setObjectName(QString::fromUtf8("BatS"));
        BatS->setGeometry(QRect(450, 30, 221, 101));
        BatS->setAutoFillBackground(false);
        BatS->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);"));
        BatS->setLineWidth(-1);
        BatS->setMidLineWidth(1);
        BatS->setNumDigits(4);
        BatS->setProperty("value", QVariant(0));
        LSpeedS = new QTextBrowser(Static);
        LSpeedS->setObjectName(QString::fromUtf8("LSpeedS"));
        LSpeedS->setGeometry(QRect(-30, -50, 171, 41));
        CoolantTempS = new QLCDNumber(Static);
        CoolantTempS->setObjectName(QString::fromUtf8("CoolantTempS"));
        CoolantTempS->setGeometry(QRect(240, 210, 159, 91));
        CoolantTempS->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);"));
        CoolantTempS->setLineWidth(-1);
        CoolantTempS->setMidLineWidth(1);
        CoolantTempS->setSmallDecimalPoint(false);
        CoolantTempS->setNumDigits(3);
        CoolantTempS->setProperty("value", QVariant(0));
        RPMS = new QLCDNumber(Static);
        RPMS->setObjectName(QString::fromUtf8("RPMS"));
        RPMS->setGeometry(QRect(140, 30, 265, 101));
        RPMS->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(27,31,34);"));
        RPMS->setLineWidth(-1);
        RPMS->setMidLineWidth(1);
        RPMS->setProperty("value", QVariant(0));
        OilTempS = new QLCDNumber(Static);
        OilTempS->setObjectName(QString::fromUtf8("OilTempS"));
        OilTempS->setGeometry(QRect(500, 200, 151, 101));
        OilTempS->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);"));
        OilTempS->setLineWidth(-1);
        OilTempS->setMidLineWidth(1);
        OilTempS->setNumDigits(3);
        OilTempS->setProperty("value", QVariant(0));
        MessageS = new QTextBrowser(Static);
        MessageS->setObjectName(QString::fromUtf8("MessageS"));
        MessageS->setGeometry(QRect(30, 340, 651, 51));
        MessageS->setStyleSheet(QString::fromUtf8("background-color: rgb(27,31,34);"));
        MessageS->setFrameShape(QFrame::NoFrame);
        MessageS->setFrameShadow(QFrame::Plain);
        MessageS->setLineWidth(0);
        tabWidget->addTab(Static, QString());
        Dynamic1 = new QWidget();
        Dynamic1->setObjectName(QString::fromUtf8("Dynamic1"));
        speedD1 = new QLCDNumber(Dynamic1);
        speedD1->setObjectName(QString::fromUtf8("speedD1"));
        speedD1->setGeometry(QRect(130, 290, 81, 81));
        speedD1->setAutoFillBackground(false);
        speedD1->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(27,31,34);"));
        speedD1->setFrameShape(QFrame::NoFrame);
        speedD1->setFrameShadow(QFrame::Plain);
        speedD1->setLineWidth(4);
        speedD1->setMidLineWidth(0);
        speedD1->setSmallDecimalPoint(false);
        speedD1->setNumDigits(2);
        speedD1->setProperty("value", QVariant(0));
        ETCD1 = new QLCDNumber(Dynamic1);
        ETCD1->setObjectName(QString::fromUtf8("ETCD1"));
        ETCD1->setGeometry(QRect(500, 290, 131, 81));
        ETCD1->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(27,31,34);"));
        ETCD1->setFrameShape(QFrame::NoFrame);
        ETCD1->setFrameShadow(QFrame::Plain);
        ETCD1->setLineWidth(0);
        ETCD1->setMidLineWidth(0);
        ETCD1->setNumDigits(3);
        ETCD1->setProperty("value", QVariant(0));
        MessageD1 = new QTextBrowser(Dynamic1);
        MessageD1->setObjectName(QString::fromUtf8("MessageD1"));
        MessageD1->setGeometry(QRect(40, 400, 651, 41));
        MessageD1->setStyleSheet(QString::fromUtf8("background-color: rgb(27,31,34);"));
        MessageD1->setFrameShape(QFrame::NoFrame);
        MessageD1->setFrameShadow(QFrame::Plain);
        MessageD1->setLineWidth(0);
        RPMD1 = new QLCDNumber(Dynamic1);
        RPMD1->setObjectName(QString::fromUtf8("RPMD1"));
        RPMD1->setGeometry(QRect(80, 20, 391, 121));
        RPMD1->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(27,31,34);"));
        RPMD1->setFrameShape(QFrame::NoFrame);
        RPMD1->setFrameShadow(QFrame::Plain);
        RPMD1->setLineWidth(0);
        RPMD1->setMidLineWidth(0);
        RPMD1->setSmallDecimalPoint(false);
        RPMD1->setNumDigits(5);
        RPMD1->setDigitCount(5);
        RPMD1->setProperty("value", QVariant(0));
        RPMD1->setProperty("intValue", QVariant(0));
        BATprogD1 = new QProgressBar(Dynamic1);
        BATprogD1->setObjectName(QString::fromUtf8("BATprogD1"));
        BATprogD1->setGeometry(QRect(580, 130, 81, 16));
        BATprogD1->setStyleSheet(QString::fromUtf8("border-color: rgb(37, 41, 44);"));
        BATprogD1->setMinimum(9);
        BATprogD1->setMaximum(14);
        BATprogD1->setValue(8);
        BATprogD1->setTextVisible(false);
        label_3 = new QLabel(Dynamic1);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(0, 0, 731, 491));
        label_3->setStyleSheet(QString::fromUtf8("background-image:\n"
"url(/home/pi/fsae/FSAEdash/DashTemplateD1.png);\n"
""));
        EXTRAD1 = new QLCDNumber(Dynamic1);
        EXTRAD1->setObjectName(QString::fromUtf8("EXTRAD1"));
        EXTRAD1->setGeometry(QRect(650, 100, 51, 16));
        EXTRAD1->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(27,31,34);"));
        EXTRAD1->setFrameShape(QFrame::NoFrame);
        EXTRAD1->setLineWidth(0);
        EXTRAD1->setMidLineWidth(0);
        EXTRAD1->setProperty("value", QVariant(0));
        label_10 = new QLabel(Dynamic1);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(670, 130, 31, 16));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Roboto"));
        font1.setPointSize(12);
        font1.setBold(false);
        font1.setItalic(false);
        font1.setWeight(9);
        label_10->setFont(font1);
        label_10->setStyleSheet(QString::fromUtf8("font: 75 12pt \"Roboto\";\n"
"font: 75 12pt \"Roboto\";\n"
"font: 75 12pt \"Roboto\";"));
        tabWidget->addTab(Dynamic1, QString());
        label_3->raise();
        speedD1->raise();
        ETCD1->raise();
        MessageD1->raise();
        RPMD1->raise();
        BATprogD1->raise();
        EXTRAD1->raise();
        label_10->raise();
        Dynamic2 = new QWidget();
        Dynamic2->setObjectName(QString::fromUtf8("Dynamic2"));
        RPMprogD2 = new QProgressBar(Dynamic2);
        RPMprogD2->setObjectName(QString::fromUtf8("RPMprogD2"));
        RPMprogD2->setGeometry(QRect(30, 300, 471, 141));
        RPMprogD2->setStyleSheet(QString::fromUtf8("selection-background-color: rgb(0, 255, 0);\n"
"background-color: rgb(27,31,34);\n"
"border-color: rgb(27, 31, 34);"));
        RPMprogD2->setMinimum(0);
        RPMprogD2->setMaximum(13000);
        RPMprogD2->setValue(100);
        RPMprogD2->setTextVisible(false);
        EXTRAprogD2 = new QProgressBar(Dynamic2);
        EXTRAprogD2->setObjectName(QString::fromUtf8("EXTRAprogD2"));
        EXTRAprogD2->setGeometry(QRect(560, 110, 111, 20));
        EXTRAprogD2->setStyleSheet(QString::fromUtf8("border-color: rgb(27, 31, 34);"));
        EXTRAprogD2->setMinimum(0);
        EXTRAprogD2->setMaximum(100);
        EXTRAprogD2->setValue(0);
        EXTRAprogD2->setTextVisible(false);
        EXTRAD2 = new QLCDNumber(Dynamic2);
        EXTRAD2->setObjectName(QString::fromUtf8("EXTRAD2"));
        EXTRAD2->setGeometry(QRect(310, 100, 121, 31));
        EXTRAD2->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(27,31,34);"));
        EXTRAD2->setFrameShape(QFrame::NoFrame);
        EXTRAD2->setLineWidth(0);
        EXTRAD2->setMidLineWidth(0);
        EXTRAD2->setProperty("value", QVariant(0));
        label_8 = new QLabel(Dynamic2);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(50, 110, 31, 16));
        label_8->setFont(font1);
        label_8->setStyleSheet(QString::fromUtf8("font: 75 12pt \"Roboto\";\n"
"font: 75 12pt \"Roboto\";"));
        MessageD2 = new QTextBrowser(Dynamic2);
        MessageD2->setObjectName(QString::fromUtf8("MessageD2"));
        MessageD2->setGeometry(QRect(30, 30, 651, 51));
        MessageD2->setStyleSheet(QString::fromUtf8("background-color: rgb(27,31,34);"));
        MessageD2->setFrameShape(QFrame::NoFrame);
        MessageD2->setFrameShadow(QFrame::Plain);
        MessageD2->setLineWidth(0);
        label_4 = new QLabel(Dynamic2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(0, 0, 711, 471));
        label_4->setStyleSheet(QString::fromUtf8("background-image:\n"
"url(/home/pi/fsae/FSAEdash/DashTemplateD2.png);"));
        speedD2 = new QLCDNumber(Dynamic2);
        speedD2->setObjectName(QString::fromUtf8("speedD2"));
        speedD2->setGeometry(QRect(100, 210, 101, 81));
        speedD2->setAutoFillBackground(false);
        speedD2->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(27,31,34);"));
        speedD2->setFrameShape(QFrame::NoFrame);
        speedD2->setFrameShadow(QFrame::Plain);
        speedD2->setLineWidth(4);
        speedD2->setMidLineWidth(0);
        speedD2->setSmallDecimalPoint(false);
        speedD2->setNumDigits(2);
        speedD2->setProperty("value", QVariant(0));
        ETCD2 = new QLCDNumber(Dynamic2);
        ETCD2->setObjectName(QString::fromUtf8("ETCD2"));
        ETCD2->setGeometry(QRect(370, 210, 131, 81));
        ETCD2->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"background-color: rgb(27,31,34);"));
        ETCD2->setFrameShape(QFrame::NoFrame);
        ETCD2->setFrameShadow(QFrame::Plain);
        ETCD2->setLineWidth(0);
        ETCD2->setMidLineWidth(0);
        ETCD2->setNumDigits(3);
        ETCD2->setProperty("value", QVariant(0));
        BATprogD2 = new QProgressBar(Dynamic2);
        BATprogD2->setObjectName(QString::fromUtf8("BATprogD2"));
        BATprogD2->setGeometry(QRect(100, 109, 131, 21));
        BATprogD2->setStyleSheet(QString::fromUtf8("border-color: rgb(27, 31, 34);"));
        BATprogD2->setMinimum(9);
        BATprogD2->setMaximum(14);
        BATprogD2->setValue(8);
        BATprogD2->setTextVisible(false);
        tabWidget->addTab(Dynamic2, QString());
        label_4->raise();
        RPMprogD2->raise();
        EXTRAprogD2->raise();
        EXTRAD2->raise();
        label_8->raise();
        MessageD2->raise();
        speedD2->raise();
        ETCD2->raise();
        BATprogD2->raise();
        Test = new QWidget();
        Test->setObjectName(QString::fromUtf8("Test"));
        ch1l = new QLabel(Test);
        ch1l->setObjectName(QString::fromUtf8("ch1l"));
        ch1l->setGeometry(QRect(60, 0, 161, 71));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Roboto"));
        font2.setPointSize(28);
        font2.setBold(false);
        font2.setItalic(false);
        font2.setWeight(3);
        ch1l->setFont(font2);
        ch1l->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"font: 25 28pt \"Roboto\";"));
        ch1l->setTextFormat(Qt::PlainText);
        ch2l = new QLabel(Test);
        ch2l->setObjectName(QString::fromUtf8("ch2l"));
        ch2l->setGeometry(QRect(370, 0, 161, 71));
        ch2l->setFont(font2);
        ch2l->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"font: 25 28pt \"Roboto\";"));
        ch2l->setTextFormat(Qt::PlainText);
        ch2 = new QLCDNumber(Test);
        ch2->setObjectName(QString::fromUtf8("ch2"));
        ch2->setGeometry(QRect(350, 70, 191, 131));
        ch2->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);"));
        ch2->setMidLineWidth(1);
        ch2->setNumDigits(5);
        ch2->setProperty("value", QVariant(298));
        ch1 = new QLCDNumber(Test);
        ch1->setObjectName(QString::fromUtf8("ch1"));
        ch1->setGeometry(QRect(40, 60, 191, 131));
        ch1->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);"));
        ch1->setMidLineWidth(1);
        ch1->setNumDigits(5);
        ch1->setProperty("value", QVariant(298));
        ch4l = new QLabel(Test);
        ch4l->setObjectName(QString::fromUtf8("ch4l"));
        ch4l->setGeometry(QRect(350, 200, 161, 71));
        ch4l->setFont(font2);
        ch4l->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"font: 25 28pt \"Roboto\";"));
        ch4l->setTextFormat(Qt::PlainText);
        ch4 = new QLCDNumber(Test);
        ch4->setObjectName(QString::fromUtf8("ch4"));
        ch4->setGeometry(QRect(350, 270, 191, 131));
        ch4->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);"));
        ch4->setMidLineWidth(1);
        ch4->setNumDigits(5);
        ch4->setProperty("value", QVariant(298));
        ch3l = new QLabel(Test);
        ch3l->setObjectName(QString::fromUtf8("ch3l"));
        ch3l->setGeometry(QRect(40, 190, 161, 71));
        ch3l->setFont(font2);
        ch3l->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"font: 25 28pt \"Roboto\";"));
        ch3l->setTextFormat(Qt::PlainText);
        ch3 = new QLCDNumber(Test);
        ch3->setObjectName(QString::fromUtf8("ch3"));
        ch3->setGeometry(QRect(30, 270, 191, 131));
        ch3->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);"));
        ch3->setMidLineWidth(1);
        ch3->setNumDigits(5);
        ch3->setProperty("value", QVariant(298));
        tabWidget->addTab(Test, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        tabWidget->addTab(tab, QString());
        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        BlabelS->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Roboto'; font-size:12pt; font-weight:200; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:16pt; color:#ffffff;\">Battery</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        LSpeedS->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Roboto'; font-size:12pt; font-weight:200; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:16pt; color:#ffffff;\">Engine Speed</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        MessageS->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Roboto'; font-size:12pt; font-weight:200; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; font-weight:600; color:#ffffff;\">messageS</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(Static), QApplication::translate("MainWindow", "Static", 0, QApplication::UnicodeUTF8));
        MessageD1->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Roboto'; font-size:12pt; font-weight:200; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; font-weight:600; color:#ffffff;\">messageD1</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        BATprogD1->setFormat(QApplication::translate("MainWindow", "%p%", 0, QApplication::UnicodeUTF8));
        label_3->setText(QString());
        label_10->setText(QApplication::translate("MainWindow", "BAT", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(Dynamic1), QApplication::translate("MainWindow", "Dynamic 1", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "BAT", 0, QApplication::UnicodeUTF8));
        MessageD2->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Roboto'; font-size:12pt; font-weight:200; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; font-weight:600; color:#ffffff;\">messageD2</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_4->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(Dynamic2), QApplication::translate("MainWindow", "Dynamic 2", 0, QApplication::UnicodeUTF8));
        ch1l->setText(QApplication::translate("MainWindow", "Channel 1", 0, QApplication::UnicodeUTF8));
        ch2l->setText(QApplication::translate("MainWindow", "Channel 2", 0, QApplication::UnicodeUTF8));
        ch4l->setText(QApplication::translate("MainWindow", "Channel 4", 0, QApplication::UnicodeUTF8));
        ch3l->setText(QApplication::translate("MainWindow", "Channel 3", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(Test), QApplication::translate("MainWindow", "Test", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
