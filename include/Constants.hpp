#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// Time Advance 0 Macro 1 milli second
#define TA_ZERO "00:00:00:020"

// Macros for LP_Reposition.hpp
#define LP_REPOSITION_TIME "00:01:00:000"
#define LAND_OUTPUT true
#define PILOT_HANDOVER true

//Macro for the distance that subsequent landing points should be separated by in meters.
#define LP_SEPARATION (10.0)

//Macros for the time in the states.
#define LZE_SCAN_TIME "00:01:00:000"
#define LP_APPROACH_TIME "00:01:00:000"
#define PLP_HANDOVER_CODE 0
#define LP_TIME_EXPIRED_CODE 0

// Landing timers
#define REPO_TIMER 60.0f //sec, timer to do reposition to new LP
#define ORBIT_TIMER 120.0 //sec, timer for orbiting
#define LP_ACCEPT_TIMER 120.0 //sec, timer to accept new LPs

// Those settings will be commanded to the FCC
#define DEFAULT_ORBIT_RADIUS 30.0 //m, this is to send to FCC to do orbit
#define DEFAULT_ORBIT_VELOCITY 2.0 //knots, this is to send to the FCC  to do orbit
#define DEFAULT_HOVER_ALTITUDE_AGL 15.0 //ft, required to send to the FCC when hovering
#define DEFAULT_ORBIT_YAW_BEHAVIOUR ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE
// This is Mavlink type to be sent to FCC when conducting orbit
#define LP_HOR_ACCEPT_TOLERANCE_DISTANCE 5.0 //meters, hor acceptance criteria to do repo
#define DEFAULT_LAND_CRITERIA_TIME 3.0 //sec, timer to meet landing criteria
#define DEFAULT_LAND_CRITERIA_HOR_DIST 16.40 // feet, to meet landing criteria
#define DEFAULT_LAND_CRITERIA_VERT_DIST 5.0  // feet , to meet landing criteria
#define DEFAULT_LAND_CRITERIA_VEL 3.0 //knots, velocity to meet landing criteria
#define DEFAULT_LAND_CRITERIA_HDG 15.0 // to meet heading criteria for landing 
#define MAX_REPO_VEL 10.0 //knots, max velocity to do reposition, used to command to the FCC

// Used in Command_Reposition. This is temporary remove when intergrating.
#define MAX_REPO_VEL 10.0 // Max speed in knots to reposition at
#define KTS_TO_MPS 0.514444 // Convert knots to m/s
#define METERS_TO_FT 3.281
#define DEFAULT_LAND_CRITERIA_HOR_DIST 16.40
#define DEFAULT_LAND_CRITERIA_VERT_DIST 5.0
#define DEFAULT_LAND_CRITERIA_VEL 3.0
#define DEFAULT_LAND_CRITERIA_HDG 15.0
#define DEFAULT_LAND_CRITERIA_TIME 3.0

#define FEET_TO_M 0.3048

// Networking
#define PEREGRINE_IP "10.1.10.2"
#define MAVLINK_OVER_UDP_PORT 14601
#define MAX_SER_BUFFER_CHARS 1024 // Given in serialToEthThreads.c 168

// Mavlink Acknowledgements
#define MAV_CMD_DEFAULT 0
#define MAV_RESULT_ACCEPTED 0
#define MAV_RESULT_TEMPORARILY_REJECTED 1
#define MAV_RESULT_DENIED 2
#define MAV_RESULT_UNSUPPORTED 3
#define MAV_RESULT_FAILED 4
#define MAV_RESULT_IN_PROGRESS 5
#define MAV_RESULT_CANCELLED 6


// Shared memory
#define DEFAULT_SHARED_MEMORY_NAME "asraSharedMem"

#endif /* CONSTANTS_HPP */