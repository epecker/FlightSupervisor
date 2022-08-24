Test Cases
=============================================================================================================================================================
| Folder | Test Path                                                                                                                         |
|--------|-----------------------------------------------------------------------------------------------------------------------------------|
| 0      | IDLE->WAIT_NEW_LP->NOTIFY_UPDATE->UPDATE_LP->NEW_LP_REPO->LP_REPO->REQUEST_LAND->LANDING_ROUTINE->PILOT_CONTROL                   |
| 1      | REQUEST_LAND->PILOT_CONTROL->WAIT_NEW_LP                                                                                          |
| 2      | IDLE->WAIT_NEW_LP->NOTIFY_UPDATE->UPDATE_LP->NEW_LP_REPO->LP_REPO->HANDOVER_CTRL->PILOT_CONTROL // Using the pilot_takeover port  |
| 3      | IDLE->WAIT_NEW_LP->NOTIFY_UPDATE->UPDATE_LP->NEW_LP_REPO->LP_REPO->HANDOVER_CTRL->PILOT_CONTROL // Using the control_yielded port |
| 4      | IDLE->WAIT_NEW_LP->NOTIFY_UPDATE->UPDATE_LP->NEW_LP_REPO->LP_REPO->NEW_LP_REPO->LP_REPO->PILOT_CONTROL                            |
| 5      | NEW_LP_REPO->PILOT_CONTROL                                                                                                        |
| 6      | IDLE->PILOT_CONTROL                                                                                                               |
