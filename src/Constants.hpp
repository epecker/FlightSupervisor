#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// Time Advance 0 Macro 1 milli second
#define TA_ZERO "00:00:00:010"

// Macros for LP_Reposition.hpp
#define LP_REPOSITION_TIME "00:01:00:000"
#define LAND_OUTPUT true
#define PILOT_HANDOVER true

//Macro for the distance that subsequent landing points should be separated by in meters.
#define LP_SEPARATION (10.0)

//Macros for the time in the states.
#define LZE_SCAN_TIME "00:02:00:000"
#define LP_APPROACH_TIME "00:02:00:000"
#define PLP_HANDOVER_CODE 0
#define LP_TIME_EXPIRED_CODE 0

// Landing timers
#define REPO_TIMER 60.0f //sec, timer to do reposition to new LP
#define ORBIT_TIMER 120.0 //sec, timer for orbiting
#define LP_ACCEPT_TIMER 120.0 //sec, timer to accept new LPs
#define UPD_TIMER 20.0 //sec, timer to wait for updated valid LPs

// Those settings will be commanded to the FCC
#define DEFAULT_ORBIT_RADIUS 30.0 //m, this is to send to FCC to do orbit
#define DEFAULT_ORBIT_VELOCITY 2.0 //knots, this is to send to the FCC  to do orbit
#define DEFAULT_HOVER_ALTITUDE_AGL 15.0 //ft, required to send to the FCC when hovering
#define DEFAULT_ORBIT_YAW_BEHAVIOUR Mav_Command_Orbit_Yaw_Behaviour_E::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE
// This is Mavlink type to be sent to FCC when conducting orbit
#define LP_HOR_ACCEPT_TOLERANCE_DISTANCE 5.0 //meters, hor acceptance criteria to do repo
#define DEFAULT_LAND_CRITERIA_TIME 3.0 //sec, timer to meet landing criteria
#define DEFAULT_LAND_CRITERIA_HOR_DIST 16.40 // feet, to meet landing criteria
#define DEFAULT_LAND_CRITERIA_VERT_DIST 5.0  // feet , to meet landing criteria
#define DEFAULT_LAND_CRITERIA_VEL 3.0 //knots, velocity to meet landing criteria
#define DEFAULT_LAND_CRITERIA_HDG 15.0 // to meet heading criteria for landing
#define MAX_REPO_VEL 5.0 //knots -- This matches the FCC value
#define MIN_REPO_VEL  1.0 //knots
#define REPO_TRANSIT_TIME 10.0f

#define KTS_TO_MPS 0.514444 // Convert knots to m/s
#define MPS_TO_KTS 1/KTS_TO_MPS // Convert m/s to knots
#define METERS_TO_FT 3.281
#define FT_TO_METERS 0.3048
#define DEFAULT_LAND_CRITERIA_HOR_DIST 16.40
#define DEFAULT_LAND_CRITERIA_VERT_DIST 5.0
#define DEFAULT_LAND_CRITERIA_VEL 3.0
#define DEFAULT_LAND_CRITERIA_HDG 15.0
#define DEFAULT_LAND_CRITERIA_TIME 3.0

// Networking
#define PEREGRINE_IP "10.1.10.2"
#define MAVLINK_OVER_UDP_PORT 14601
#define MAX_SER_BUFFER_CHARS 1024 // Given in serialToEthThreads.c 168

// Mavlink Acknowledgements
#define MAV_CMD_DEFAULT 0
#define MAV_RESULT_ACCEPTED 0

// Shared memory
#define DEFAULT_SHARED_MEMORY_NAME "asraSharedMem"

#define WPT_PREVIEW_LENGTH 3

// Mavlink defines
#define MAVLINK_CORE_HEADER_LEN 9
#define MAVLINK_MSG_ID_STATUSTEXT_LEN 54
#define MAVLINK_MSG_ID_STATUSTEXT 253
#define MAVLINK_STX 253
#define MY_MAV_SYS_ID 1
#define MY_MAV_COMP_ID 1
#define MAVLINK_MSG_ID_STATUSTEXT_CRC 83
#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_NUM_CHECKSUM_BYTES 2

#define SIG_ID_SET_MISSION_MONITOR_STATUS 1
#define SIG_ID_MISSION_COMPLETE 2
#define SIG_ID_MISSION_ITEM_REACHED 3
#define SIG_ID_START_MISSION 4
#define SIG_ID_LANDING_POINT 5

// Network Addresses & Ports
#define LOCALHOST "127.0.0.1"
#define BROADCAST_AIRCRAFT "132.246.193.255"
// #define HOST "192.168.101.106"
#define HOST "10.0.0.63"
#define IPV4_BOSS HOST
#define IPV4_FCC HOST
#define IPV4_GCS HOST
#define IPV4_QGC_BROADCAST BROADCAST_AIRCRAFT
#define IPV4_MAVNRC HOST

#define PORT_BOSS 13333
#define PORT_FCC 4060
#define PORT_GCS 14550
#define PORT_MAVNRC 24000
#define PORT_QGC_BROADCAST 12346

#endif /* CONSTANTS_HPP */
