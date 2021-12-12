Test Cases
=============================================================================================================================================================
Folder	| Target Output			| Input Sequence
--------| ---------------------	| --------------------------------------------------------------------------------------------------------------------------------
0	    | LP_expired			| lp_recv(00:00:10)
1	    | start_LZE_scan		| plp_ach(00:00:10)
2	    | mission_complete		| lp_recv(00:00:10)->aircraft_state(00:00:15)->landing_achieved(00:00:20)
3	    | land_requested		| lp_recv(00:00:10)->aircraft_state(00:00:15)
4	    | fcc_command_velocity	| lp_recv(00:00:10)->aircraft_state(00:00:15)
5	    | control_yielded		| lp_recv(00:00:10)->pilot_takeover(00:02:13)
6	    | notify_pilot			| lp_recv(00:00:10)
7		| fcc_command_hover		| lp_recv(00:00:10)