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
    Waypoint.h \
    UBConfig.h \
    UBPlanner.h \

SOURCES += \
    main.cpp \
    Waypoint.cc \
    UBPlanner.cpp \

INCLUDEPATH += \
    mavlink/include/mavlink/v2.0 \
    mavlink/include/mavlink/v2.0/ardupilotmega \
