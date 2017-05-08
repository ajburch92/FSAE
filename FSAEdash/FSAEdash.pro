#-------------------------------------------------
#
# Project created by QtCreator 2016-01-19T17:44:24
#
#-------------------------------------------------

QT+= core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = FSAEdash
TEMPLATE = app

INCLUDEPATH += /usr/local/include

SOURCES += main.cpp\
        mainwindow.cpp \
    MAIN2.cpp \
    daq.cpp

LIBS +=-L/usr/local/lib -lwiringPi

QMAKE_CXXFLAGS += -std=c++0x

HEADERS  += mainwindow.h \
    MAIN2.h \
    structures.h \
    daq.h

FORMS    += mainwindow.ui
