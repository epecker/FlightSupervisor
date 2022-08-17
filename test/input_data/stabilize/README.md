Test Cases
=====================================================================================================
| Folder | Test Path                                                                                                                                                    |
|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0      | IDLE->MISSION_STARTED->REQUEST_AIRCRAFT_STATE->GET_AIRCRAFT_STATE->INIT_HOVER->STABILIZING->CHECK_STATE->STABILIZING->...->HOVER->MISSION_STARTED (Internal) |
| 1      | HOVER->MISSION_STARTED (External)                                                                                                                            |
| 2      | IDLE->MISSION_STARTED->REQUEST_AIRCRAFT_STATE->GET_AIRCRAFT_STATE->INIT_HOVER->STABILIZING->CHECK_STATE->STABILIZING->...->MISSION_STARTED                   |
| 3      | CHECK_STATE->MISSION_STARTED                                                                                                                                 |
| 4      | IDLE->MISSION_STARTED->REQUEST_AIRCRAFT_STATE->GET_AIRCRAFT_STATE->INIT_HOVER->STABILIZING->MISSION_STARTED                                                  |
| 5      | INIT_HOVER->MISSION_STARTED                                                                                                                                  |
