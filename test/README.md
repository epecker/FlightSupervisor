Testing
=====================================
This README has details on how testing of the Supervisor can be conducted using the provided test drivers.

Testing in Faster than Real-Time
-------------------------------------
The following test drivers can be used to test the DEVS models using batches of input files in faster than real-time:
- td_command_reposition
- td_handover_control
- td_landing_routine
- td_lp_manager
- td_lp_reposition
- td_reposition_timer
- td_stabilize
- td_supervisor

To add tests to the test set for each driver:
1. Open the test/input_data/<Test Driver Name> directory
2. Create a folder and name it 1 larger than the largest existing folder.
3. Copy the files from an existing test into the new folder.
4. Go through each file and add inputs according to the input reader format.
5. Run the associated test driver then view the results.

Testing in Real-Time
-------------------------------------
The following test drivers can be run in real-time on UNIX platforms:
- td_blocking_input
- td_command_line_input
- td_supervisor_command_line

To use these drivers simply run the .EXE and follow the instructions on the console.

