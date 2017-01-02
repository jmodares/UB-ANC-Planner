#ifndef CONFIG_H
#define CONFIG_H

#define COMM_RANGE 50
#define VISUAL_RANGE 10

#define MAV_PORT  5760
#define PHY_PORT 15760
#define SNR_PORT 25760
#define PWR_PORT 35760

#define PXY_PORT 45760

#define AGENT_FILE "./agent"
#define MISSION_FILE "/mission.txt"
#define FIRMWARE_FILE "./firmware"

#define OBJECTS_PATH "./objects"

#define ENGINE_TRACK_RATE 1000
#define OBJECT_TRACK_RATE 1000
#define SERVER_TRACK_RATE 1000

#define PACKET_END "\r\r\n\n"
#define BROADCAST_ID 255

//#define PHY_PORT 52001
#define PHY_TRACK_RATE 1000
#define SNR_TRACK_RATE 1000

#define SERIAL_PORT "ttyACM0"
#define BAUD_RATE 115200

#define GND_RES 17

#define GPS_ACCURACY 5
#define POINT_ZONE 1
#define TAKEOFF_ALT 5
#define MISSION_START_DELAY 10000
#define MISSION_TRACK_RATE 1000

#endif // CONFIG_H
