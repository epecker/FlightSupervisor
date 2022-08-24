# Testing
This README has details on how testing of the Supervisor can be conducted using the provided test drivers.

## Contents

* [Testing in Faster than Real-Time](#testing-in-faster-than-real-time)
* [Testing in Real-Time](#testing-in-real-time)
* [The Command Line Landing Test Driver](#the-command-line-landing-test-driver)
* [Preparing the Test Outputs for Review](#preparing-the-test-outputs-for-review)
  * [Prerequisites](#prerequisites)
  * [Perform the Tests](#perform-the-tests)

## Testing in Faster than Real-Time
The following test drivers can be used to test the DEVS models using batches of input files in faster than real-time:
- td_command_reposition
- td_handover_control
- td_landing_routine
- td_lp_manager
- td_lp_reposition
- td_reposition_timer
- td_stabilize
- td_landing

To add tests to the test set for each driver:
1. Open the test/input_data/<Test Driver Name> directory
2. Create a folder and name it 1 larger than the largest existing folder.
3. Copy the files from an existing test into the new folder.
4. Go through each file and add inputs according to the input reader format.
5. Run the associated test driver then view the results.

## Testing in Real-Time
The following test drivers can be run in real-time on UNIX platforms:
- td_blocking_input
- td_command_line_input
- td_landing_command_line

To use these drivers simply run the .EXE and follow the instructions on the console.

## The Command Line Landing Test Driver
The command line Landing test driver (td_landing_command_line.exe) is a user driven method of testing the Landing phase in real-time. The driver works by receiving port value pairs from the user on the command line and forwarding them as event messages to the Landing Model's input ports. The user will continually be prompted for input until 'q' is entered. To send a message to a certain port on the Landing model, akin to how it will receive messages via a network, the user specifies which port the message should be sent to:

	0. The landing achieved port which takes a single boolean (0/1) value.
	1. The aircraft state port which takes an aircraft state structure as a value.
	2. The pilot takeover port which takes a single boolean (0/1) value.
	3. The landing point received port which takes a mavlink mission item structure input.
	4. The planned landing point achieved port which takes a mavlink mission item structure input.

The description of each of the ports can be found in the port description document. The members of each of the structures can be found in either the source code or on the terminal during test execution.
Example test cases can be found here [Command Line Landing Test Driver Example Test Cases](input_data/landing_command_line/README.md)

## Preparing the test outputs for review

### Prerequisites

* Install python 3
* Install tabulate for python 3
  * pip install tabulate
  * python3 -m pip install tabulate
* Add python to your system environment path

### Perform the Tests

1. Execute at least one of the test drivers(td) with the name td_*.exe
	* This will create log files in `<Project_Directory>/test/simulation_results/<Model_Name>/<Test_Number>`
2. Open a terminal
3. Navigate to the project directory
	```
	cd <Project_Directory>
	```
4. Run the python Script
	```
	python ./test/scripts/simulation_cleanup.py ./<simulation_results>
	```
5. The output can be found in `<Project_Directory>/test/simulation_results/<Model_Name>.md`
