#-------------------------------------------------
#
# Project created by QtCreator 2016-07-01T10:26:57
#
#-------------------------------------------------

QT       += core

QT       += gui

TARGET   = ub-anc-planner
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

#
# APM Planner Library
#
include(apm_planner.pri)

#
# CPLEX Optimization Library
#
include(cplex.pri)

INCLUDEPATH += \

HEADERS += \
    UBPlanner.h \
    config.h \

SOURCES += \
    UBPlanner.cpp \
