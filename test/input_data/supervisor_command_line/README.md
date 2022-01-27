Command Line Supervisor Test Driver
==========================================
Below are example test cases that use the command line Supervisor test driver. The test driver is run in real-time and can accept any input messages that the Supervisor on board the aircraft would be expected to handle. Each test case shows the inputs that should be supplied to the Supervisor but the state changes and output messages can only be viewed after the simulation has ended in the simulation output folder.

Nominal Test Case
------------------------------------------
This test demonstrates a nominal trajectory through the SUpervisor from the initial idle state to receipt of the mission complete signal:
1. The first step in the test is to supply a landing point as shown in the message below.
	```
	3 1 45.6 -75.4 100.0
	```
2. Then once a landing point has been received, an aircraft state is required to form the command to traverse to the landing point. Using the aircraft state the Supervisor can request a velocity from the FCC then tell it to reach a hover criteria over the landing point. 
	'''
	1 45.6 -75.4 100.0 90.0 10.0
	```
3. Then the user should wait 3 seconds to ensure that the Supervisor has reached a hover criteria (3 seconds is the default hover time tolerance). After the three seconds has elapsed a message should be sent informing the Supervisor that the aircraft has landed.
	```
	0 1
	```
4. The last step is to enter the message `q` then wait for the simulation to terminate at 120 seconds.

Orbit Test Case
------------------------------------------
This test demonstrates that the Supervisor will send a command to start orbiting the planned landing point:
1. The first step in the test is to inform the Supervisor that it has achieved the planned landing point.
	```
	4 1 45.6 -75.4 100.0
	```
2. Then the user should wait 3 seconds to ensure that the Supervisor has reached a hover criteria over the landing zone (3 seconds is the default hover time tolerance). After the three seconds has elapsed an output will be generated starting the landing zone evaluation scan (the user will not see this output until checking the log file).
3. Once the scan has commenced the user will have 60 seconds to enter a landing point before the scan timer expires and a handover occurs.
	```
	3 1 45.6 -75.4 100.0
	```
4. Once a landing point has been received, an aircraft state is required to form the command to traverse to the landing point. Using the aircraft state the Supervisor can request a velocity from the FCC then tell it to reach a hover criteria over the landing point. 
	'''
	1 45.6 -75.4 100.0 90.0 10.0
	```
5. Then the user should wait 3 seconds to ensure that the Supervisor has reached a hover criteria (3 seconds is the default hover time tolerance). After the three seconds has elapsed a message should be sent informing the Supervisor that the aircraft has landed.
	```
	0 1
	```
6. The last step is to enter the message `q` then wait for the simulation to terminate at 120 seconds.

Handover Test Case
------------------------------------------
This test demonstrates the process of a pilot handover in the case that a critical timer has expired.
1. The first step in the test is to supply a landing point which will start the landing point reposition timer.
	```
	3 1 45.6 -75.4 100.0
	```
2. The user should then wait 60 seconds for the timer to expire, triggering a handover sequence.
3. After a further 3 seconds have elapsed (for the Supervisor to achieve a hover criteria) the user should supply a pilot takeover message.
	```
	2 1
	```
4. The last step is to enter the message `q` then wait for the simulation to terminate at 120 seconds.
