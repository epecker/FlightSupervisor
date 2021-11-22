// C++ headers
#include <string>
#include <filesystem>

// Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>

// Time class header
#include <NDTime.hpp>

// Required for testing
#include "../../include/SupervisorConfig.hpp" // Generated by cmake.
#include "../../include/input_readers.hpp" // Input Reader Definitions.

// Atomic model to test.
#include "../../include/atomic_models/Reposition_Timer.hpp" 

using TIME = NDTime;
using namespace std;

// Used for oss_sink_state and oss_sink_messages
ofstream out_messages;
ofstream out_state;

// Model output ports
struct o_pilot_handover : public out_port<bool> {};
struct o_land : public out_port<bool> {};
struct o_request_reposition : public out_port<message_mavlink_mission_item_t> {};

int main(int argc, char* argv[]) {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/reposition_timer/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/reposition_timer/");

	do {
		// Input locations
		string input_dir					= i_base_dir + to_string(test_set_enumeration);
		string input_file_lp_new			= input_dir + string("/lp_new.txt");
		string input_file_pilot_takeover	= input_dir + string("/pilot_takeover.txt");
		string input_file_lp_criteria_met	= input_dir + string("/lp_criteria_met.txt");
		string input_file_control_yielded	= input_dir + string("/control_yielded.txt");

		// Output locations
		string out_directory				= o_base_dir + to_string(test_set_enumeration);
		string out_messages_file			= out_directory + string("/output_messages.txt");
		string out_state_file				= out_directory + string("/output_state.txt");

		if (!filesystem::exists(input_file_lp_new) ||
			!filesystem::exists(input_file_pilot_takeover) ||
			!filesystem::exists(input_file_lp_criteria_met) ||
			!filesystem::exists(input_file_control_yielded)
			) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		filesystem::create_directories(out_directory.c_str());

		// Instantiate the atomic model to test
		shared_ptr<dynamic::modeling::model> reposition_timer = dynamic::translate::make_dynamic_atomic_model<Reposition_Timer, TIME>("reposition_timer");

		// Instantiate the input readers.
		// One for each input
		shared_ptr<dynamic::modeling::model> ir_lp_new =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_lp_new", move(input_file_lp_new.c_str()));
		shared_ptr<dynamic::modeling::model> ir_pilot_takeover =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_pilot_takeover", move(input_file_pilot_takeover.c_str()));
		shared_ptr<dynamic::modeling::model> ir_lp_crit_met =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_lp_crit_met", move(input_file_lp_criteria_met.c_str()));
		shared_ptr<dynamic::modeling::model> ir_control_yielded =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_control_yielded", move(input_file_control_yielded.c_str()));

		// The models to be included in this coupled model 
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
			ir_lp_new,
			ir_pilot_takeover,
			ir_lp_crit_met,
			ir_control_yielded,
			reposition_timer
		};

		dynamic::modeling::Ports iports_TestDriver = {	};

		dynamic::modeling::Ports oports_TestDriver = {
			typeid(o_pilot_handover),
			typeid(o_land),
			typeid(o_request_reposition)
		};

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = {
			dynamic::translate::make_EOC<Reposition_Timer_defs::o_pilot_handover,o_pilot_handover>("reposition_timer"),
			dynamic::translate::make_EOC<Reposition_Timer_defs::o_land,o_land>("reposition_timer"),
			dynamic::translate::make_EOC<Reposition_Timer_defs::o_request_reposition,o_request_reposition>("reposition_timer")
		};

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
			dynamic::translate::make_IC<iestream_input_defs<message_mavlink_mission_item_t>::out,Reposition_Timer_defs::i_lp_new>("ir_lp_new", "reposition_timer"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out,Reposition_Timer_defs::i_pilot_takeover>("ir_pilot_takeover", "reposition_timer"),
			dynamic::translate::make_IC<iestream_input_defs<message_mavlink_mission_item_t>::out,Reposition_Timer_defs::i_lp_crit_met>("ir_lp_crit_met", "reposition_timer"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out,Reposition_Timer_defs::i_control_yielded>("ir_control_yielded", "reposition_timer")
		};

		shared_ptr<dynamic::modeling::coupled<TIME>> TEST_DRIVER = make_shared<dynamic::modeling::coupled<TIME>>(
			"TEST_DRIVER", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
		);

		/*************** Loggers *******************/
		out_messages = ofstream(out_messages_file);
		struct oss_sink_messages {
			static ostream& sink() {
				return out_messages;
			}
		};

		out_state = ofstream(out_state_file);
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
		test_set_enumeration++;
	} while (filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

	return 0;
}
