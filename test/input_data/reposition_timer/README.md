Test Cases
=============================================================================================================================================================
Folder	| Test Path
--------| ---------------------------------------------------------------------------------------------------------------------------------------------	
0		| IDLE->NEW_LP_REPO->LP_REPO->REQUEST_LAND->LANDING_ROUTINE->PILOT_CONTROL
1		| IDLE->NEW_LP_REPO->LP_REPO->REQUEST_LAND->PILOT_CONTROL
2		| IDLE->NEW_LP_REPO->LP_REPO->HANDOVER_CTRL->PILOT_CONTROL // Using the pilot_takeover port
3		| IDLE->NEW_LP_REPO->LP_REPO->HANDOVER_CTRL->PILOT_CONTROL // Using the control_yielded port
4		| IDLE->NEW_LP_REPO->LP_REPO->NEW_LP_REPO->LP_REPO->PILOT_CONTROL
5		| IDLE->NEW_LP_REPO->PILOT_CONTROL
6		| IDLE->PILOT_CONTROL
