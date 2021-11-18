//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "../../include/message_structures/aircraft_state_message.hpp"
#include "../../include/message_structures/hover_criteria_message.hpp"
#include "../../include/message_structures/lp_message.hpp"
#include "../../include/message_structures/fcc_command.hpp"

//Atomic model headers
#include <cadmium/basic_model/pdevs/iestream.hpp> //Atomic model for inputs
#include "../../include/atomic_models/Command_Reposition.hpp"

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

// LPMessage input reader
template<typename T>
class IR_LPMessage_t : public iestream_input<LPMessage_t, T> {
public:
	IR_LPMessage_t() = default;
	IR_LPMessage_t(const char* file_path) : iestream_input<LPMessage_t, T>(file_path) {};
};

// AircraftStateMessage input reader
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
struct o_cancel_hover : public out_port<bool> {};
struct o_fcc_command_velocity : public out_port<FccCommandMessage_t> {};
struct o_lp_criteria_met : public out_port<LPMessage_t> {};
struct o_stabilize : public out_port<HoverCriteriaMessage_t> {};


/**
* ==========================================================
* MAIN METHOD
* ==========================================================
*/
int main(int argc, char* argv[]) {
	// Input Files
	const string input_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/command_reposition/");
	const string input_file_aircraft_state = input_dir + string("aircraft_state.txt");
	const string input_file_hover_criteria_met = input_dir + string("hover_criteria_met.txt");
	const string input_file_pilot_handover = input_dir + string("pilot_handover.txt");
	const string input_file_pilot_takeover = input_dir + string("pilot_takeover.txt");
	const string input_file_request_reposition = input_dir + string("request_reposition.txt");

	if (!filesystem::exists(input_file_aircraft_state) ||
		!filesystem::exists(input_file_hover_criteria_met) ||
		!filesystem::exists(input_file_pilot_handover) ||
		!filesystem::exists(input_file_pilot_takeover) ||
		!filesystem::exists(input_file_request_reposition))
	{
		printf("One of the input files do not exist\n");
		return 1;
	}

	// Instantiate the atomic model to test
	shared_ptr<dynamic::modeling::model> command_reposition = dynamic::translate::make_dynamic_atomic_model<Command_Reposition, TIME>("command_reposition");

	// Instantiate the input readers.
	// One for each input
	shared_ptr<dynamic::modeling::model> ir_aircraft_state = 
		dynamic::translate::make_dynamic_atomic_model<IR_AircraftStateMessage_t, TIME, const char* >("ir_aircraft_state", move(input_file_aircraft_state.c_str()));
	shared_ptr<dynamic::modeling::model> ir_hover_criteria_met =
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_hover_criteria_met", move(input_file_hover_criteria_met.c_str()));
	shared_ptr<dynamic::modeling::model> ir_pilot_handover = 
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_pilot_handover", move(input_file_pilot_handover.c_str()));
	shared_ptr<dynamic::modeling::model> ir_pilot_takeover = 
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_pilot_takeover", move(input_file_pilot_takeover.c_str()));
	shared_ptr<dynamic::modeling::model> ir_request_reposition = 
		dynamic::translate::make_dynamic_atomic_model<IR_LPMessage_t, TIME, const char* >("ir_request_reposition", move(input_file_request_reposition.c_str()));

	// The models to be included in this coupled model 
	// (accepts atomic and coupled models)
	dynamic::modeling::Models submodels_TestDriver = {
		ir_aircraft_state,
		ir_hover_criteria_met,
		ir_pilot_handover,
		ir_pilot_takeover,
		ir_request_reposition,
		command_reposition
	};

	dynamic::modeling::Ports iports_TestDriver = {	};

	dynamic::modeling::Ports oports_TestDriver = {
		typeid(o_cancel_hover),
		typeid(o_fcc_command_velocity),
		typeid(o_lp_criteria_met),
		typeid(o_stabilize)
	};

	dynamic::modeling::EICs eics_TestDriver = {	};

	// The output ports will be used to export in logging
	dynamic::modeling::EOCs eocs_TestDriver = {
		dynamic::translate::make_EOC<Command_Reposition_defs::o_cancel_hover,o_cancel_hover>("command_reposition"),
		dynamic::translate::make_EOC<Command_Reposition_defs::o_fcc_command_velocity,o_fcc_command_velocity>("command_reposition"),
		dynamic::translate::make_EOC<Command_Reposition_defs::o_stabilize,o_stabilize>("command_reposition"),
		dynamic::translate::make_EOC<Command_Reposition_defs::o_lp_criteria_met,o_lp_criteria_met>("command_reposition")
	};
	
	// This will connect our outputs from our input reader to the file
	dynamic::modeling::ICs ics_TestDriver = {
		dynamic::translate::make_IC<iestream_input_defs<AircraftStateMessage_t>::out,Command_Reposition_defs::i_aircraft_state>("ir_aircraft_state", "command_reposition"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,Command_Reposition_defs::i_hover_criteria_met>("ir_hover_criteria_met", "command_reposition"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,Command_Reposition_defs::i_pilot_handover>("ir_pilot_handover", "command_reposition"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,Command_Reposition_defs::i_pilot_takeover>("ir_pilot_takeover", "command_reposition"),
		dynamic::translate::make_IC<iestream_input_defs<LPMessage_t>::out,Command_Reposition_defs::i_request_reposition>("ir_request_reposition", "command_reposition")
	};

	shared_ptr<dynamic::modeling::coupled<TIME>> TEST_DRIVER = make_shared<dynamic::modeling::coupled<TIME>>(
		"TEST_DRIVER", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
	);

	/*************** Loggers *******************/
	string out_directory = string(PROJECT_DIRECTORY) + string("/test/simulation_results/command_reposition/");
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