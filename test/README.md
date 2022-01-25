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

The Command Line Supervisor Test Driver
=====================================
The command line Supervisor test driver (td_supervisor_command_line.exe) is a user driven method of testing the Supervisor in real-time. The driver works by receiving port value pairs from the user on the command line and forwarding them as event messsages to the Supervisors input ports. The user will continually be prompted for input until 'q' is entered. To send a message to a certain port on the Supervisor, akin to how it will receive messages via a network, the user specifies which port the message should be sent to:
0. The landing achieved port which takes a single boolean (0/1) value.
1. The aircraft state port which takes an aircraft state structure as a value.
2. The pilot takeover port which takes a single boolean (0/1) value.
3. The landing point received port which takes a mavlink mission item structure input.
4. The planned landing point achieved port which takes a mavlink mission item structure input.
The description of each of the ports can be found in the port description document. The members of each of the structures can be found in either the source code or on the terminal during test execution.
