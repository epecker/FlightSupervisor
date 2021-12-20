CC		= g++
CFLAGS	= -std=c++17 
MACROS 	= -D CADMIUM_EXECUTE_CONCURRENT -D RT_LINUX -D RT_DEVS -D MISSED_DEADLINE_TOLERANCE=-10 -D BOOST_THREAD_PROVIDES_FUTURE_CONTINUATION -D BOOST_THREAD_PROVIDES_EXECUTORS -D BOOST_THREAD_USES_MOVE

INCLUDECADMIUM		= -I ../cadmium/include
INCLUDERTCADMIUM	= -I ../cadmium/include
INCLUDEDESTIMES		= -I ../cadmium/DESTimes/include
INCLUDEBOOST		= -I /usr/include/boost
LINKBOOSTLLOC		= -L/usr/include/boost
LINKBOOSTLIBS		= -lboost_system -lboost_thread

#CREATu BIN AND BUILD FOLDERS TO SAVE THE COMPILED FILES DURING RUNTIME
bin_folder := $(shell mkdir -p bin)
build_folder := $(shell mkdir -p build)
results_folder := $(shell mkdir -p simulation_results)

#TARGET TO COMPILE ALL THE TESTS TOGETHER 
td_blocking_input.o: test/drivers/td_blocking_input.cpp
	$(CC) -c $(CFLAGS) $(MACROS) $(INCLUDEBOOST) $(INCLUDERTCADMIUM) $(INCLUDEDESTIMES) test/drivers/td_blocking_input.cpp $(LINKBOOSTLLOC) $(LINKBOOSTLIBS) -o build/td_blocking_input.o

tests: td_blocking_input.o
	$(CC) build/td_blocking_input.o -o bin/td_blocking_input

#TARGET TO COMPILE ONLY REAL-TIME SUPERVISOR
rt_supervisor.o: src/rt_supervisor.cpp
	$(CC) -c $(CFLAGS) $(MACROS) $(INCLUDEBOOST) $(INCLUDERTCADMIUM) $(INCLUDEDESTIMES) src/rt_supervisor.cpp $(LINKBOOSTLLOC) $(LINKBOOSTLIBS) -o build/rt_supervisor.o 
	
simulator: rt_supervisor.o 
	$(CC) -o bin/rt_supervisor build/rt_supervisor.o 

#TARGET TO COMPILE EVERYTHING
all: simulator tests

#CLEAN COMMANDS
clean:
	rm -f bin/* build/*
