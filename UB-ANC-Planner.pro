#-------------------------------------------------
#
# Project created by QtCreator 2016-07-01T10:26:57
#
#-------------------------------------------------

QT       += core

QT       += gui
QT       += positioning

TARGET   = ub-anc-planner
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

#
# CPLEX Optimization Library
#
include(cplex.pri)

HEADERS += \
    UBConfig.h \
    UBPlanner.h \
    Waypoint.h \

SOURCES += \
    UBPlanner.cpp \
    Waypoint.cc \
    main.cpp \

INCLUDEPATH += \
    mavlink/include/mavlink/v2.0 \
    mavlink/include/mavlink/v2.0/ardupilotmega \
