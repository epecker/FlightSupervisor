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
#include "../../include/coupled_models/LP_Reposition.hpp" 

using TIME = NDTime;
using namespace std;

// Used for oss_sink_state and oss_sink_messages
ofstream out_messages;
ofstream out_state;

// Model output ports
struct o_cancel_hover : public out_port<bool> {};
struct o_fcc_command_velocity : public out_port<message_fcc_command_t> {};
struct o_land_requested : public out_port<bool> {};
struct o_mission_complete : public out_port<bool> {};
struct o_pilot_handover : public out_port<message_mavlink_mission_item_t> {};
struct o_stabilize : public out_port<message_hover_criteria_t> {};

int main(int argc, char* argv[]) {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/lp_reposition/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/lp_reposition/");

	do {
		// Input locations
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_aircraft_state = input_dir + string("/aircraft_state.txt");
		string input_file_control_yielded = input_dir + string("/control_yielded.txt");
		string input_file_hover_criteria_met = input_dir + string("/hover_criteria_met.txt");
		string input_file_landing_achieved = input_dir + string("/landing_achieved.txt");
		string input_file_lp_new = input_dir + string("/lp_new.txt");
		string input_file_pilot_takeover = input_dir + string("/pilot_takeover.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");

		if (!filesystem::exists(input_file_aircraft_state) ||
			!filesystem::exists(input_file_control_yielded) ||
			!filesystem::exists(input_file_hover_criteria_met) ||
			!filesystem::exists(input_file_landing_achieved) ||
			!filesystem::exists(input_file_lp_new) ||
			!filesystem::exists(input_file_pilot_takeover)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		filesystem::create_directories(out_directory.c_str());

		// Instantiate the atomic model to test
		LP_Reposition lpr = LP_Reposition();
		shared_ptr<dynamic::modeling::coupled<TIME>> lp_reposition = make_shared<dynamic::modeling::coupled<TIME>>("lp_reposition", lpr.submodels, lpr.iports, lpr.oports, lpr.eics, lpr.eocs, lpr.ics);

		// Instantiate the input readers.
		// One for each input
		shared_ptr<dynamic::modeling::model> ir_aircraft_state =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Aircraft_State, TIME, const char* >("ir_aircraft_state", move(input_file_aircraft_state.c_str()));
		shared_ptr<dynamic::modeling::model> ir_control_yielded =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_control_yielded", move(input_file_control_yielded.c_str()));
		shared_ptr<dynamic::modeling::model> ir_hover_criteria_met =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_hover_criteria_met", move(input_file_hover_criteria_met.c_str()));
		shared_ptr<dynamic::modeling::model> ir_landing_achieved =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_landing_achieved", move(input_file_landing_achieved.c_str()));
		shared_ptr<dynamic::modeling::model> ir_lp_new =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_lp_new", move(input_file_lp_new.c_str()));
		shared_ptr<dynamic::modeling::model> ir_pilot_takeover =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_pilot_takeover", move(input_file_pilot_takeover.c_str()));
		
		// The models to be included in this coupled model 
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
			lp_reposition,
			ir_aircraft_state,
			ir_control_yielded,
			ir_hover_criteria_met,
			ir_landing_achieved,
			ir_lp_new,
			ir_pilot_takeover
		};

		dynamic::modeling::Ports iports_TestDriver = {	};

		dynamic::modeling::Ports oports_TestDriver = {
			typeid(o_cancel_hover),
			typeid(o_fcc_command_velocity),
			typeid(o_land_requested),
			typeid(o_mission_complete),
			typeid(o_pilot_handover),
			typeid(o_stabilize)
		};

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = {
			dynamic::translate::make_EOC<LP_Reposition_defs::o_cancel_hover,o_cancel_hover>("lp_reposition"),
			dynamic::translate::make_EOC<LP_Reposition_defs::o_fcc_command_velocity,o_fcc_command_velocity>("lp_reposition"),
			dynamic::translate::make_EOC<LP_Reposition_defs::o_land_requested,o_land_requested>("lp_reposition"),
			dynamic::translate::make_EOC<LP_Reposition_defs::o_mission_complete,o_mission_complete>("lp_reposition"),
			dynamic::translate::make_EOC<LP_Reposition_defs::o_pilot_handover,o_pilot_handover>("lp_reposition"),
			dynamic::translate::make_EOC<LP_Reposition_defs::o_stabilize,o_stabilize>("lp_reposition")
		};

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
			dynamic::translate::make_IC<iestream_input_defs<message_aircraft_state_t>::out,LP_Reposition_defs::i_aircraft_state>("ir_aircraft_state", "lp_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out,LP_Reposition_defs::i_control_yielded>("ir_control_yielded", "lp_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out,LP_Reposition_defs::i_hover_criteria_met>("ir_hover_criteria_met", "lp_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out,LP_Reposition_defs::i_landing_achieved>("ir_landing_achieved", "lp_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<message_mavlink_mission_item_t>::out,LP_Reposition_defs::i_LP_new>("ir_lp_new", "lp_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out,LP_Reposition_defs::i_pilot_takeover>("ir_pilot_takeover", "lp_reposition")
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

	fcloseall();
	fflush(NULL);
	string path_to_script = PROJECT_DIRECTORY + string("/test/scripts/simulation_cleanup.py");
	string path_to_simulation_results = PROJECT_DIRECTORY + string("/test/simulation_results");
	if (system("python3 --version") == 0) {
		string command = "python3 " + path_to_script + string(" ") + path_to_simulation_results;
		system(command.c_str());
	} else if (system("python --version") == 0) {
		string command = "python " + path_to_script + string(" ") + path_to_simulation_results;
		system(command.c_str());
	} else {
		cout << "\nPython is not installed!\n";
	}

	return 0;
}
