# APM Planner 2 Console Version

# Qt configuration
CONFIG += \
    thread

QT += network \
    serialport \

##  testlib is needed even in release flavor for QSignalSpy support
QT += testlib

#
# MAVLink
#
MAVLINK_CONF = ""
MAVLINKPATH = apm_planner/libs/mavlink/include/mavlink/v1.0
DEFINES += MAVLINK_NO_DATA

# If the user config file exists, it will be included.
# if the variable MAVLINK_CONF contains the name of an
# additional project, QGroundControl includes the support
# of custom MAVLink messages of this project. It will also
# create a QGC_USE_{AUTOPILOT_NAME}_MESSAGES macro for use
# within the actual code.
exists(user_config.pri) {
    include(user_config.pri)
    message("----- USING CUSTOM USER QGROUNDCONTROL CONFIG FROM user_config.pri -----")
    message("Adding support for additional MAVLink messages for: " $$MAVLINK_CONF)
    message("------------------------------------------------------------------------")
} else {
    MAVLINK_CONF += ardupilotmega
}
INCLUDEPATH += $$MAVLINKPATH
isEmpty(MAVLINK_CONF) {
    INCLUDEPATH += $$MAVLINKPATH/common
message("Using mavlink common")
} else {
    INCLUDEPATH += $$MAVLINKPATH/$$MAVLINK_CONF
message("Using mavlink " + $$MAVLINK_CONF)
    DEFINES += $$sprintf('QGC_USE_%1_MESSAGES', $$upper($$MAVLINK_CONF))
}

#
# Logging Library
#
include (apm_planner/QsLog/QsLog.pri)

#
# AGLLIB Math Library
#
include(apm_planner/libs/alglib/alglib.pri)
DEFINES += NOMINMAX

INCLUDEPATH += \
    apm_planner/libs/alglib/src \

#
# OpenPilot GCS Library
#
DEFINES += EXTERNAL_USE

INCLUDEPATH += \
    apm_planner/libs/opmapcontrol/src/internals/projections \

SOURCES += \
    apm_planner/libs/opmapcontrol/src/core/point.cpp \
    apm_planner/libs/opmapcontrol/src/core/size.cpp \
    apm_planner/libs/opmapcontrol/src/internals/pointlatlng.cpp \
    apm_planner/libs/opmapcontrol/src/internals/pureprojection.cpp \
    apm_planner/libs/opmapcontrol/src/internals/projections/mercatorprojection.cpp \

INCLUDEPATH += \
    apm_planner/src/ \
    apm_planner/src/comm \
    apm_planner/src/uas \

SOURCES += \
    apm_planner/src/comm/AbsPositionOverview.cc \
    apm_planner/src/comm/LinkInterface.cpp \
    apm_planner/src/comm/LinkManager.cc \
    apm_planner/src/comm/LinkManagerFactory.cpp \
    apm_planner/src/comm/MAVLinkDecoder.cc \
    apm_planner/src/comm/MAVLinkProtocol.cc \
    apm_planner/src/comm/MissionOverview.cc \
    apm_planner/src/comm/QGCFlightGearLink.cc \
    apm_planner/src/comm/QGCJSBSimLink.cc \
    apm_planner/src/comm/RelPositionOverview.cc \
    apm_planner/src/comm/serialconnection.cc \
    apm_planner/src/comm/TCPLink.cc \
    apm_planner/src/comm/UASObject.cc \
    apm_planner/src/comm/UDPClientLink.cc \
    apm_planner/src/comm/UDPLink.cc \
    apm_planner/src/comm/VehicleOverview.cc \
    apm_planner/src/uas/ArduPilotMegaMAV.cc \
    apm_planner/src/uas/PxQuadMAV.cc \
    apm_planner/src/uas/QGCMAVLinkUASFactory.cc \
    apm_planner/src/uas/senseSoarMAV.cpp \
    apm_planner/src/uas/SlugsMAV.cc \
    apm_planner/src/uas/UAS.cc \
    apm_planner/src/uas/UASManager.cc \
    apm_planner/src/uas/UASParameter.cpp \
    apm_planner/src/uas/UASWaypointManager.cc \
    apm_planner/src/globalobject.cc \
    apm_planner/src/LogCompressor.cc \
    apm_planner/src/main.cc \
    apm_planner/src/QGC.cc \
    apm_planner/src/QGCCore.cc \
    apm_planner/src/QGCGeo.cc \
    apm_planner/src/Waypoint.cc \

HEADERS += \
    apm_planner/src/comm/AbsPositionOverview.h \
    apm_planner/src/comm/LinkInterface.h \
    apm_planner/src/comm/LinkManager.h \
    apm_planner/src/comm/LinkManagerFactory.h \
    apm_planner/src/comm/MAVLinkDecoder.h \
    apm_planner/src/comm/MAVLinkProtocol.h \
    apm_planner/src/comm/MissionOverview.h \
    apm_planner/src/comm/ProtocolInterface.h \
    apm_planner/src/comm/QGCFlightGearLink.h \
    apm_planner/src/comm/QGCHilLink.h \
    apm_planner/src/comm/QGCJSBSimLink.h \
    apm_planner/src/comm/QGCMAVLink.h \
    apm_planner/src/comm/RelPositionOverview.h \
    apm_planner/src/comm/serialconnection.h \
    apm_planner/src/comm/SerialLinkInterface.h \
    apm_planner/src/comm/TCPLink.h \
    apm_planner/src/comm/UASObject.h \
    apm_planner/src/comm/UDPClientLink.h \
    apm_planner/src/comm/UDPLink.h \
    apm_planner/src/comm/VehicleOverview.h \
    apm_planner/src/uas/ArduPilotMegaMAV.h \
    apm_planner/src/uas/PxQuadMAV.h \
    apm_planner/src/uas/QGCMAVLinkUASFactory.h \
    apm_planner/src/uas/senseSoarMAV.h \
    apm_planner/src/uas/SlugsMAV.h \
    apm_planner/src/uas/UAS.h \
    apm_planner/src/uas/UASInterface.h \
    apm_planner/src/uas/UASManager.h \
    apm_planner/src/uas/UASParameter.h \
    apm_planner/src/uas/UASWaypointManager.h \
    apm_planner/src/configuration.h \
    apm_planner/src/globalobject.h \
    apm_planner/src/LogCompressor.h \
    apm_planner/src/MG.h \
    apm_planner/src/QGC.h \
    apm_planner/src/QGCCore.h \
    apm_planner/src/QGCGeo.h \
    apm_planner/src/Waypoint.h \
