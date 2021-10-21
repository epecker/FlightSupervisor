//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "../data_structures/lp_message.hpp"
#include "../data_structures/plp_message.hpp"

//Atomic model headers
#include <cadmium/basic_model/pdevs/iestream.hpp> //Atomic model for inputs
#include "../atomic_models/LP_Manager.hpp"

// Project information headers this is created by cmake at generation time!!!!
#include "../include/SupervisorConfig.hpp"

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

// PLPMessage input reader
template<typename T>
class IR_PLPMessage_t : public iestream_input<PLPMessage_t, T> {
public:
	IR_PLPMessage_t() = default;
	IR_PLPMessage_t(const char* file_path) : iestream_input<PLPMessage_t, T>(file_path) {};
};

// Bool input reader
template<typename T>
class IR_Boolean : public iestream_input<bool, T> {
public:
	IR_Boolean() = default;
	IR_Boolean(const char* file_path) : iestream_input<bool, T>(file_path) {};
};

// Define output ports to be used for logging purposes
struct o_lp_new : public out_port<LPMessage_t> {};
struct o_lp_expired : public out_port<LPMessage_t> {};
struct o_pilot_handover : public out_port<bool> {};
struct o_stabilize : public out_port<bool> {};
struct o_start_lze_scan : public out_port<bool> {};

/**
* ==========================================================
* MAIN METHOD
* ==========================================================
*/
int main(int argc, char* argv[]) {
	// Input Files
	const string input_dir = string(PROJECT_DIRECTORY) + string("/input_data/lp_manager/");
	const string input_file_lp_recv = input_dir + string("lp_recv.txt");
	const string input_file_plp_ach = input_dir + string("plp_ach.txt");
	const string input_file_pilot_takeover = input_dir + string("pilot_takeover.txt");
	const string input_file_hover_criteria_met = input_dir + string("hover_criteria_met.txt");
	const string input_file_control_yielded = input_dir + string("control_yielded.txt");

	if (!filesystem::exists(input_file_lp_recv) ||
		!filesystem::exists(input_file_plp_ach) ||
		!filesystem::exists(input_file_pilot_takeover) ||
		!filesystem::exists(input_file_hover_criteria_met) ||
		!filesystem::exists(input_file_control_yielded)
		) {
		printf("One of the input files do not exist\n");
		return 1;
	}

	// Instantiate the atomic model to test
	shared_ptr<dynamic::modeling::model> lp_manager = dynamic::translate::make_dynamic_atomic_model<LP_Manager, TIME>("lp_manager");

	// Instantiate the input readers.
	// One for each input
	shared_ptr<dynamic::modeling::model> ir_lp_recv = 
		dynamic::translate::make_dynamic_atomic_model<IR_LPMessage_t, TIME, const char* >("ir_lp_recv", move(input_file_lp_recv.c_str()));
	shared_ptr<dynamic::modeling::model> ir_plp_ach = 
		dynamic::translate::make_dynamic_atomic_model<IR_PLPMessage_t, TIME, const char* >("ir_plp_ach", move(input_file_plp_ach.c_str()));
	shared_ptr<dynamic::modeling::model> ir_pilot_takeover = 
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_pilot_takeover", move(input_file_pilot_takeover.c_str()));
	shared_ptr<dynamic::modeling::model> ir_hover_criteria_met = 
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_hover_criteria_met", move(input_file_hover_criteria_met.c_str()));
	shared_ptr<dynamic::modeling::model> ir_control_yielded = 
		dynamic::translate::make_dynamic_atomic_model<IR_Boolean, TIME, const char* >("ir_control_yielded", move(input_file_control_yielded.c_str()));

	// The models to be included in this coupled model 
	// (accepts atomic and coupled models)
	dynamic::modeling::Models submodels_TestDriver = {
		ir_lp_recv,
		ir_plp_ach,
		ir_pilot_takeover,
		ir_hover_criteria_met,
		ir_control_yielded,
		lp_manager
	};

	dynamic::modeling::Ports iports_TestDriver = {	};

	dynamic::modeling::Ports oports_TestDriver = {
		typeid(o_lp_new),
		typeid(o_lp_expired),
		typeid(o_pilot_handover),
		typeid(o_stabilize),
		typeid(o_start_lze_scan)
	};

	dynamic::modeling::EICs eics_TestDriver = {	};

	// The output ports will be used to export in logging
	dynamic::modeling::EOCs eocs_TestDriver = {
		dynamic::translate::make_EOC<LP_Manager_defs::o_lp_new,o_lp_new>("lp_manager"),
		dynamic::translate::make_EOC<LP_Manager_defs::o_lp_expired,o_lp_expired>("lp_manager"),
		dynamic::translate::make_EOC<LP_Manager_defs::o_pilot_handover,o_pilot_handover>("lp_manager"),
		dynamic::translate::make_EOC<LP_Manager_defs::o_stabilize,o_stabilize>("lp_manager"),
		dynamic::translate::make_EOC<LP_Manager_defs::o_start_lze_scan,o_start_lze_scan>("lp_manager")
	};
	
	// This will connect our outputs from our input reader to the file
	dynamic::modeling::ICs ics_TestDriver = {
		dynamic::translate::make_IC<iestream_input_defs<LPMessage_t>::out,LP_Manager_defs::i_lp_recv>("ir_lp_recv", "lp_manager"),
		dynamic::translate::make_IC<iestream_input_defs<PLPMessage_t>::out,LP_Manager_defs::i_plp_ach>("ir_plp_ach", "lp_manager"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,LP_Manager_defs::i_pilot_takeover>("ir_pilot_takeover", "lp_manager"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,LP_Manager_defs::i_hover_criteria_met>("ir_hover_criteria_met", "lp_manager"),
		dynamic::translate::make_IC<iestream_input_defs<bool>::out,LP_Manager_defs::i_control_yielded>("ir_control_yielded", "lp_manager")
	};

	shared_ptr<dynamic::modeling::coupled<TIME>> TEST_DRIVER = make_shared<dynamic::modeling::coupled<TIME>>(
		"TEST_DRIVER", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
	);

	/*************** Loggers *******************/
	string out_directory = string(PROJECT_DIRECTORY) + string("/simulation_results/lp_manager/");
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