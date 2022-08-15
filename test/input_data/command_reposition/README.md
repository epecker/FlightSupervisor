Test Cases
==========================================================================================================================================================
| Folder | Test Path                                                                                                                       |
|--------|---------------------------------------------------------------------------------------------------------------------------------|
| 0      | IDLE->MISSION_START->REQUEST_STATE->GET_STATE->COMMAND_VEL->COMMAND_HOVER->STABILIZING->LP_CRITERIA_MET->LANDING->PILOT_CONTROL |
| 1      | IDLE->MISSION_START->REQUEST_STATE->GET_STATE->COMMAND_VEL->COMMAND_HOVER->STABILIZING->LP_CRITERIA_MET->LANDING->TIMER_EXPIRED |
| 2      | IDLE->MISSION_START->REQUEST_STATE->GET_STATE->COMMAND_VEL->COMMAND_HOVER->STABILIZING->CANCEL_HOVER->REQUEST_STATE->GET_STATE  |
| 3      | IDLE->MISSION_START->REQUEST_STATE->GET_STATE->COMMAND_VEL->COMMAND_HOVER->STABILIZING->PILOT_CONTROL                           |
| 4      | IDLE->MISSION_START->REQUEST_STATE->GET_STATE->COMMAND_VEL->COMMAND_HOVER->STABILIZING->TIMER_EXPIRED                           |
| 5      | IDLE->MISSION_START->REQUEST_STATE->GET_STATE->PILOT_CONTROL                                                                    |
| 6      | IDLE->MISSION_START->REQUEST_STATE->GET_STATE->TIMER_EXPIRED                                                                    |
| 7      | IDLE->MISSION_START->PILOT_CONTROL                                                                                              |
| 8      | IDLE->MISSION_START->TIMER_EXPIRED                                                                                              |
| 9      | LP_CRITERIA_MET->CANCEL_HOVER->REQUEST_STATE->GET_STATE                                                                         |
| 10     | CANCEL_HOVER->PILOT_CONTROL                                                                                                     |
| 11     | CANCEL_HOVER->TIMER_EXPIRED                                                                                                     |
| 12     | LP_CRITERIA_MET->PILOT_CONTROL                                                                                                  |
| 13     | LP_CRITERIA_MET->TIMER_EXPIRED                                                                                                  |
| 14     | COMMAND_HOVER->REQUEST_STATE->GET_STATE                                                                                         |
| 15     | COMMAND_HOVER->PILOT_CONTROL                                                                                                    |
| 16     | COMMAND_HOVER->TIMER_EXPIRED                                                                                                    |
| 17     | COMMAND_VEL->REQUEST_STATE->GET_STATE                                                                                           |
| 18     | COMMAND_VEL->PILOT_CONTROL                                                                                                      |
| 19     | COMMAND_VEL->TIMER_EXPIRED                                                                                                      |
| 20     | COMMAND_VEL->MISSION_START                                                                                                      |
