Test Cases
=====================================================================================================
| Folder	 | Test Path                                                                                                                        |
|---------|----------------------------------------------------------------------------------------------------------------------------------|
| 0			    | IDLE->REQUEST_AIRCRAFT_STATE->GET_AIRCRAFT_STATE->INIT_HOVER->STABILIZING->CHECK_STATE->STABILIZING->...->HOVER->IDLE (Internal) |
| 1			    | HOVER->IDLE (External)                                                                                                           |
| 2			    | IDLE->REQUEST_AIRCRAFT_STATE->GET_AIRCRAFT_STATE->INIT_HOVER->STABILIZING->CHECK_STATE->STABILIZING->...->IDLE                   |
| 3			    | CHECK_STATE->IDLE                                                                                                                |
| 4			    | IDLE->REQUEST_AIRCRAFT_STATE->GET_AIRCRAFT_STATE->INIT_HOVER->STABILIZING->IDLE                                                  |
| 5			    | INIT_HOVER->IDLE                                                                                                                 |
