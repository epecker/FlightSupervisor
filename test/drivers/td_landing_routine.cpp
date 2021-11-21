//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "../../include/message_structures/message_hover_criteria_t.hpp"

//Atomic model headers
#include <cadmium/basic_model/pdevs/iestream.hpp> //Atomic model for inputs
#include "../../include/atomic_models/Landing_Routine.hpp"

// Project information headers this is created by cmake at generation time!!!!
#include "../../include/SupervisorConfig.hpp"

//C++ headers
#include <chrono>
#include <algorithm>
#include <string>
#include <iostream>
#include <filesystem>

using namespace std;
using namespace cadmium;
using namespace cadmium::basic_models::pdevs;

using TIME = NDTime;

/**
* ==========================================================
* INPUT READERS
* ==========================================================
*/

// Bool input reader
template<typename T>
class IR_Boolean : public iestream_input<bool, T> {
public:
	IR_Boolean() = default;
	IR_Boolean(const char* file_path) : iestream_input<bool, T>(file_path) {};
};

// Define output ports to be used for logging purposes
struct o_land_requested : public out_port<bool> {};
struct o_mission_complete : public out_port<bool> {};
struct o_stabilize : public out_port<message_hover_criteria_t> {};

/**
* ==========================================================
* MAIN METHOD
* ==========================================================
*/
int main(int argc, char* argv[]) {
	// Input Files
	const string input_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/landing_routine/");
	const string input_file_landing_achieved = input_dir + string("landing_achieved.txt");
	const string input_file_pilot_takeover = input_dir + string("pilot_takeover.txt");
	const string input_file_hover_criteria_met = input_dir + string("hover_criteria_met.txt");
	const string input_file_land = input_dir + string("land.txt");

	if (!filesystem::exists(input_file_landing_achieved) ||
		!filesystem::exists(input_file_pilot_takeover) ||
		!filesystem::exists(input_file_hover_criteria_met) ||
		!filesystem::exists(input_file_land)
		) {
		printf("One of the input files do not exist\n");
		return 1;
	}

	// Instantiate the atomic model to test
	shared_ptr<dynamic::modeling::model> landing_routine = dynamic::translate::make_dynamic_atomic_model<Landing_Routine, TIME>("landing_routine");

	// Instantiate the input readers.
	// One for each input
	shared_ptr<dynamic::modeling::model> ir_landing_achieved =
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_landing_achieved", move(input_file_landing_achieved.c_str()));
	shared_ptr<dynamic::modeling::model> ir_pilot_takeover =
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_pilot_takeover", move(input_file_pilot_takeover.c_str()));
	shared_ptr<dynamic::modeling::model> ir_hover_criteria_met =
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_hover_criteria_met", move(input_file_hover_criteria_met.c_str()));
	shared_ptr<dynamic::modeling::model> ir_land =
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_land", move(input_file_land.c_str()));

	// The models to be included in this coupled model 
	// (accepts atomic and coupled models)
	dynamic::modeling::Models submodels_TestDriver = {
		ir_landing_achieved,
		ir_pilot_takeover,
		ir_hover_criteria_met,
		ir_land,
		landing_routine
	};

	dynamic::modeling::Ports iports_TestDriver = {	};

	dynamic::modeling::Ports oports_TestDriver = {
		typeid(o_land_requested),
		typeid(o_mission_complete),
		typeid(o_stabilize)
	};

	dynamic::modeling::EICs eics_TestDriver = {	};

	// The output ports will be used to export in logging
	dynamic::modeling::EOCs eocs_TestDriver = {
		dynamic::translate::make_EOC<Landing_Routine_defs::o_land_requested,o_land_requested>("landing_routine"),
		dynamic::translate::make_EOC<Landing_Routine_defs::o_mission_complete,o_mission_complete>("landing_routine"),
		dynamic::translate::make_EOC<Landing_Routine_defs::o_stabilize,o_stabilize>("landing_routine")
	};
	
	// This will connect our outputs from our input reader to the file
	dynamic::modeling::ICs ics_TestDriver = {
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,Landing_Routine_defs::i_landing_achieved>("ir_landing_achieved", "landing_routine"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,Landing_Routine_defs::i_pilot_takeover>("ir_pilot_takeover", "landing_routine"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,Landing_Routine_defs::i_hover_crit_met>("ir_hover_criteria_met", "landing_routine"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,Landing_Routine_defs::i_land>("ir_land", "landing_routine")
	};

	shared_ptr<dynamic::modeling::coupled<TIME>> TEST_DRIVER = make_shared<dynamic::modeling::coupled<TIME>>(
		"TEST_DRIVER", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
	);

	/*************** Loggers *******************/
	string out_directory = string(PROJECT_DIRECTORY) + string("/test/simulation_results/landing_routine/");
	string out_messages_file = out_directory + string("output_messages.txt");
	string out_state_file = out_directory + string("output_state.txt");

	filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

	static ofstream out_messages(out_messages_file.c_str());
	struct oss_sink_messages {
		static ostream& sink() {
			return out_messages;
		}
	};
	static ofstream out_state(out_state_file.c_str());
	struct oss_sink_state {
		static ostream& sink() {
			return out_state;
		}
	};

	using state = logger::logger<logger::logger_state, dynamic::logger::formatter<TIME>, oss_sink_state>;
	using log_messages = logger::logger<logger::logger_messages, dynamic::logger::formatter<TIME>, oss_sink_messages>;
	using global_time_mes = logger::logger<logger::logger_global_time, dynamic::logger::formatter<TIME>, oss_sink_messages>;
	using global_time_sta = logger::logger<logger::logger_global_time, dynamic::logger::formatter<TIME>, oss_sink_state>;

	using logger_supervisor = logger::multilogger<state, log_messages, global_time_mes, global_time_sta>;

	dynamic::engine::runner<NDTime, logger_supervisor> r(TEST_DRIVER, { 0 });

	r.run_until_passivate();
	return 0;
}