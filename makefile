CC		= g++
CFLAGS	= -std=c++17 CADMIUM_EXECUTE_CONCURRENT RT_LINUX RT_DEVS MISSED_DEADLINE_TOLERANCE=-10 

INCLUDECADMIUM	= -I C:/Cadmium-Simulation-Environment/cadmium/include
INCLUDECADMIUM	= -I ../../cadmium/include
INCLUDEDESTIMES	= -I C:/Cadmium-Simulation-Environment/DESTimes/include



#CREATE BIN AND BUILD FOLDERS TO SAVE THE COMPILED FILES DURING RUNTIME
bin_folder := $(shell mkdir -p bin)
build_folder := $(shell mkdir -p build)
results_folder := $(shell mkdir -p simulation_results)

#TARGET TO COMPILE ALL THE TESTS TOGETHER 
td_blocking_input.o: test/drivers/td_blocking_input.cpp
	$(CC) -c $(CFLAGS) $(INCLUDECADMIUM) $(INCLUDEDESTIMES) test/drivers/td_blocking_input.cpp -o build/td_blocking_input.o

tests: Test_Filter.o Test_FlightControl.o Test_Generator.o Test_GeneratorSwitch.o Test_Handover.o Test_Pilot.o Test_Reader.o Test_Reposition.o Test_SupervisorTestDriver.o 
	$(CC) build/td_blocking_input.o -o bin/td_blocking_input

#TARGET TO COMPILE ONLY REAL-TIME SUPERVISOR
rt_supervisor.o: src/rt_supervisor.cpp
	$(CC) -c $(CFLAGS) $(INCLUDECADMIUM) $(INCLUDEDESTIMES) src/rt_supervisor.cpp -o build/rt_supervisor.o
	
simulator: rt_supervisor.o 
	$(CC) -o bin/rt_supervisor build/rt_supervisor.o 

#TARGET TO COMPILE EVERYTHING
all: simulator tests

#CLEAN COMMANDS
clean:
	rm -f bin/* build/*
