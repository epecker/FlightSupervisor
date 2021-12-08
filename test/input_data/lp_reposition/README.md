Test Cases
=============================================================================================================================================================
Folder	| Inputs and Expected Outputs
--------| ---------------------------------------------------------------------------------------------------------------------------------------------
0	    | Input: <br>lp_new(00:00:10)->aircraft_state(00:00:15)->hover_criteria_met(00:00:20)->landing_achieved(00:00:25) <br> Output: <br>fcc_command_velocity(00:00:15:001)->stabilize(00:00:15:002)->land_requested(00:00:20:003)->mission_complete(00:00:25:001)
1	    | Input: <br>lp_new(00:00:10)->control_yielded(00:02:10) <br> Output:<br> pilot_handover(00:02:00)
2	    | Input: <br>lp_new(00:00:10)->aircraft_state(00:00:15)->lp_new(00:00:20)->aircraft_state(00:00:25) ->hover_criteria_met(00:00:30)->landing_achieved(00:00:35)<br> Output: <br>fcc_command_velocity(00:00:15:001)->stabilize(00:00:15:002)->cancel_hover(00:00:20:002)->fcc_command_velocity(00:00:30:001)->stabilize(00:00:30:002)->mission_complete(00:00:35:001)
3	    | Input: <br>lp_new(00:00:10)->aircraft_state(00:00:15)->lp_new(00:00:20)->aircraft_state(00:00:25) ->hover_criteria_met(00:00:30)->pilot_takeover(00:00:40)<br> Output: <br>fcc_command_velocity(00:00:15:001)->stabilize(00:00:15:002)->cancel_hover(00:00:20:002)->fcc_command_velocity(00:00:30:001)->stabilize(00:00:30:002)
4	    | Input: <br>lp_new(00:00:10)->pilot_control(00:00:40) <br> Output:<br> N/A
5	    | Input: <br>lp_new(00:00:10)->pilot_control(00:00:40)->landing_achieved(00:00:45) <br> Output:<br> mission_complete(00:00:45:001)
6	    | Input: <br>pilot_control(00:00:40) <br> Output:<br>