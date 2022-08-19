// C++ headers
#include <string>
#include <filesystem>

// Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>

// Time class header
#include <NDTime.hpp>

// Required for testing
#include "SupervisorConfig.hpp" // Generated by cmake.
#include "input_readers.hpp" // Input Reader Definitions.

// Atomic model to test.
#include "atomic_models/Command_Reposition.hpp"

using TIME = NDTime;

int main() {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/command_reposition/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/command_reposition/");

	do {
		// Input locations
		string input_dir						= i_base_dir + to_string(test_set_enumeration);
		string input_file_initial_state			= input_dir + string("/initial_state.txt");
		string input_file_aircraft_state		= input_dir + string("/aircraft_state.txt");
		string input_file_hover_criteria_met	= input_dir + string("/hover_criteria_met.txt");
		string input_file_pilot_handover		= input_dir + string("/pilot_handover.txt");
		string input_file_pilot_takeover		= input_dir + string("/pilot_takeover.txt");
		string input_file_request_reposition	= input_dir + string("/request_reposition.txt");
		string input_file_start_mission     	= input_dir + string("/start_mission.txt");

		// Output locations
		string out_directory					= o_base_dir + to_string(test_set_enumeration);
		string out_messages_file				= out_directory + string("/output_messages.txt");
		string out_state_file					= out_directory + string("/output_state.txt");

		if (!filesystem::exists(input_file_initial_state) ||
			!filesystem::exists(input_file_aircraft_state) ||
			!filesystem::exists(input_file_hover_criteria_met) ||
			!filesystem::exists(input_file_pilot_handover) ||
			!filesystem::exists(input_file_pilot_takeover) ||
			!filesystem::exists(input_file_request_reposition) ||
			!filesystem::exists(input_file_start_mission)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

        // Read the initial state of the model.
        fstream f;
        f.open(input_file_initial_state, ios::in);
        if (!f.is_open()) {
            printf("Failed to open initial_state file\n");
            return 1;
        }
        string initial_state_string;
        getline(f, initial_state_string);
        f.close();

		Command_Reposition<TIME>::States initial_state = Command_Reposition<TIME>::stringToEnum(initial_state_string);

		// Create the output location
		filesystem::create_directories(out_directory.c_str());

		// Instantiate the atomic model to test
		shared_ptr<dynamic::modeling::model> command_reposition = dynamic::translate::make_dynamic_atomic_model<Command_Reposition, TIME, Command_Reposition<TIME>::States>("command_reposition", std::move(initial_state));

		// Instantiate the input readers.
		// One for each input
		shared_ptr<dynamic::modeling::model> ir_aircraft_state =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Aircraft_State, TIME, const char* >("ir_aircraft_state", input_file_aircraft_state.c_str());
		shared_ptr<dynamic::modeling::model> ir_hover_criteria_met =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_hover_criteria_met", input_file_hover_criteria_met.c_str());
		shared_ptr<dynamic::modeling::model> ir_pilot_handover =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_pilot_handover", input_file_pilot_handover.c_str());
		shared_ptr<dynamic::modeling::model> ir_pilot_takeover =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_pilot_takeover", input_file_pilot_takeover.c_str());
		shared_ptr<dynamic::modeling::model> ir_request_reposition =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_request_reposition", input_file_request_reposition.c_str());
        shared_ptr<dynamic::modeling::model> ir_start_mission =
                dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_start_mission", input_file_start_mission.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
			ir_aircraft_state,
			ir_hover_criteria_met,
			ir_pilot_handover,
			ir_pilot_takeover,
			ir_request_reposition,
            ir_start_mission,
			command_reposition
		};

		dynamic::modeling::Ports iports_TestDriver = { };

		dynamic::modeling::Ports oports_TestDriver = { };

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = { };

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
			dynamic::translate::make_IC<iestream_input_defs<message_aircraft_state_t>::out,Command_Reposition<TIME>::defs::i_aircraft_state>("ir_aircraft_state", "command_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out,Command_Reposition<TIME>::defs::i_hover_criteria_met>("ir_hover_criteria_met", "command_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<message_landing_point_t>::out,Command_Reposition<TIME>::defs::i_pilot_handover>("ir_pilot_handover", "command_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out,Command_Reposition<TIME>::defs::i_pilot_takeover>("ir_pilot_takeover", "command_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<message_landing_point_t>::out,Command_Reposition<TIME>::defs::i_request_reposition>("ir_request_reposition", "command_reposition"),
			dynamic::translate::make_IC<iestream_input_defs<int>::out,Command_Reposition<TIME>::defs::i_start_mission>("ir_start_mission", "command_reposition")
		};

		shared_ptr<dynamic::modeling::coupled<TIME>> TEST_DRIVER = make_shared<dynamic::modeling::coupled<TIME>>(
			"TEST_DRIVER", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
		);

		/*************** Loggers *******************/
        static ofstream out_messages;
        static ofstream out_state;

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

	fflush(nullptr);
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
