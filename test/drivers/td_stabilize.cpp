// C++ headers
#include <string>
#include <filesystem>

// Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>

// Time class header
#include <NDTime.hpp>

//Messages structures
#include "message_structures/message_fcc_command_t.hpp"
#include "message_structures/message_hover_criteria_t.hpp"

// Required for testing
#include "SupervisorConfig.hpp" // Generated by cmake
#include "input_readers.hpp" // Input Reader Definitions

//Atomic model headers
#include "atomic_models/Stabilize.hpp"

using namespace std;
using namespace cadmium;
using namespace cadmium::basic_models::pdevs;

using TIME = NDTime;

/**
* ==========================================================
* MAIN METHOD
* ==========================================================
*/
int main() {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/stabilize/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/stabilize/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_initial_state = input_dir + string("/initial_state.txt");
		string input_file_aircraft_state = input_dir + string("/aircraft_state.txt");
		string input_file_cancel_hover = input_dir + string("/cancel_hover.txt");
		string input_file_stabilize = input_dir + string("/stabilize.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");

		if (!filesystem::exists(input_file_initial_state) ||
			!filesystem::exists(input_file_aircraft_state) ||
			!filesystem::exists(input_file_cancel_hover) ||
			!filesystem::exists(input_file_stabilize)
			) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Read the initial state of the model.
		string initial_state_string;
		fstream f;
		f.open(input_file_initial_state, ios::in);
		if (!f.is_open()) {
			printf("Failed to open initial_state file\n");
			return 1;
		}
		getline(f, initial_state_string);
		f.close();

		Stabilize<TIME>::States initial_state = Stabilize<TIME>::stringToEnum(initial_state_string);

		// Create the output location
		filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the atomic model to test
		shared_ptr<dynamic::modeling::model> stabilize = dynamic::translate::make_dynamic_atomic_model<Stabilize, TIME, Stabilize<TIME>::States>("stabilize", move(initial_state));

		// Instantiate the input readers.
		// One for each input
		shared_ptr<dynamic::modeling::model> ir_aircraft_state =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Aircraft_State, TIME, const char* >("ir_aircraft_state", input_file_aircraft_state.c_str());
		shared_ptr<dynamic::modeling::model> ir_cancel_hover =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_cancel_hover", input_file_cancel_hover.c_str());
		shared_ptr<dynamic::modeling::model> ir_stabilize =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Hover_Criteria, TIME, const char* >("ir_stabilize", input_file_stabilize.c_str());

		// The models to be included in this coupled model 
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
			ir_aircraft_state,
			ir_cancel_hover,
			ir_stabilize,
			stabilize
		};

		dynamic::modeling::Ports iports_TestDriver = { };

		dynamic::modeling::Ports oports_TestDriver = { };

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = { };

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
			dynamic::translate::make_IC<iestream_input_defs<message_aircraft_state_t>::out,Stabilize<TIME>::defs::i_aircraft_state>("ir_aircraft_state", "stabilize"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out,Stabilize<TIME>::defs::i_cancel_hover>("ir_cancel_hover", "stabilize"),
			dynamic::translate::make_IC<iestream_input_defs<message_hover_criteria_t>::out,Stabilize<TIME>::defs::i_stabilize>("ir_stabilize", "stabilize")
		};

		shared_ptr<dynamic::modeling::coupled<TIME>> test_driver = make_shared<dynamic::modeling::coupled<TIME>>(
			"test_driver", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
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

		dynamic::engine::runner<NDTime, logger_supervisor> r(test_driver, { 0 });

		r.run_until(TIME("00:02:00:000"));
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