//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "../../include/message_structures/lp_message.hpp"
#include "../../include/message_structures/plp_message.hpp"
#include "../../include/message_structures/fcc_command.hpp"
#include "../../include/message_structures/hover_criteria_message.hpp"

//Atomic model headers
#include <cadmium/basic_model/pdevs/iestream.hpp> //Atomic model for inputs
#include "../../include/atomic_models/Stabilize.hpp"

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

// HoverCriteriaMessage input reader
template<typename T>
class IR_HoverCriteriaMessage_t : public iestream_input<HoverCriteriaMessage_t, T> {
public:
	IR_HoverCriteriaMessage_t() = default;
	IR_HoverCriteriaMessage_t(const char* file_path) : iestream_input<HoverCriteriaMessage_t, T>(file_path) {};
};

// LPMessage input reader
template<typename T>
class IR_AircraftStateMessage_t : public iestream_input<AircraftStateMessage_t, T> {
public:
	IR_AircraftStateMessage_t() = default;
	IR_AircraftStateMessage_t(const char* file_path) : iestream_input<AircraftStateMessage_t, T>(file_path) {};
};

// Bool input reader
template<typename T>
class IR_Boolean : public iestream_input<bool, T> {
public:
	IR_Boolean() = default;
	IR_Boolean(const char* file_path) : iestream_input<bool, T>(file_path) {};
};

// Define output ports to be used for logging purposes
struct o_fcc_command_hover : public out_port<FccCommandMessage_t> {};
struct o_hover_criteria_met : public out_port<bool> {};

/**
* ==========================================================
* MAIN METHOD
* ==========================================================
*/
int main(int argc, char* argv[]) {
	// Input Files
	const string input_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/stabilize/");
	const string input_file_aircraft_state = input_dir + string("aircraft_state.txt");
	const string input_file_cancel_hover = input_dir + string("cancel_hover.txt");
	const string input_file_stabilize = input_dir + string("stabilize.txt");

	if (!filesystem::exists(input_file_aircraft_state) ||
		!filesystem::exists(input_file_cancel_hover) ||
		!filesystem::exists(input_file_stabilize)
		) {
		printf("One of the input files do not exist\n");
		return 1;
	}

	// Instantiate the atomic model to test
	shared_ptr<dynamic::modeling::model> stabilize = dynamic::translate::make_dynamic_atomic_model<Stabilize, TIME>("stabilize");

	// Instantiate the input readers.
	// One for each input
	shared_ptr<dynamic::modeling::model> ir_aircraft_state = 
		dynamic::translate::make_dynamic_atomic_model<IR_AircraftStateMessage_t, TIME, const char* >("ir_aircraft_state", move(input_file_aircraft_state.c_str()));
	shared_ptr<dynamic::modeling::model> ir_cancel_hover =
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_cancel_hover", move(input_file_cancel_hover.c_str()));
	shared_ptr<dynamic::modeling::model> ir_stabilize = 
		dynamic::translate::make_dynamic_atomic_model<IR_HoverCriteriaMessage_t, TIME, const char* >("ir_stabilize", move(input_file_stabilize.c_str()));

	// The models to be included in this coupled model 
	// (accepts atomic and coupled models)
	dynamic::modeling::Models submodels_TestDriver = {
		ir_aircraft_state,
		ir_cancel_hover,
		ir_stabilize,
		stabilize
	};

	dynamic::modeling::Ports iports_TestDriver = {	};

	dynamic::modeling::Ports oports_TestDriver = {
		typeid(o_hover_criteria_met),
		typeid(o_fcc_command_hover)
	};

	dynamic::modeling::EICs eics_TestDriver = {	};

	// The output ports will be used to export in logging
	dynamic::modeling::EOCs eocs_TestDriver = {
		dynamic::translate::make_EOC<Stabilize_defs::o_hover_criteria_met,o_hover_criteria_met>("stabilize")
	};
	
	// This will connect our outputs from our input reader to the file
	dynamic::modeling::ICs ics_TestDriver = {
		dynamic::translate::make_IC<iestream_input_defs<AircraftStateMessage_t>::out,Stabilize_defs::i_aircraft_state>("ir_aircraft_state", "stabilize"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,Stabilize_defs::i_cancel_hover>("ir_cancel_hover", "stabilize"),
		dynamic::translate::make_IC<iestream_input_defs<HoverCriteriaMessage_t>::out,Stabilize_defs::i_stabilize>("ir_stabilize", "stabilize")
	};

	shared_ptr<dynamic::modeling::coupled<TIME>> TEST_DRIVER = make_shared<dynamic::modeling::coupled<TIME>>(
		"TEST_DRIVER", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
	);

	/*************** Loggers *******************/
	string out_directory = string(PROJECT_DIRECTORY) + string("/test/simulation_results/stabilize/");
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