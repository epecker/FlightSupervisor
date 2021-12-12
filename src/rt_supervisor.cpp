#ifndef RT_LINUX
#define RT_LINUX
#endif

#ifndef CADMIUM_EXECUTE_CONCURRENT
#define CADMIUM_EXECUTE_CONCURRENT
#endif

// #ifndef DEBUG_SCHEDULING
// #define DEBUG_SCHEDULING
// #endif

#ifndef MISSED_DEADLINE_TOLERANCE
#define MISSED_DEADLINE_TOLERANCE -10
#endif

//C++ headers
#include <chrono>
#include <algorithm>
#include <string>
#include <iostream>
#include <filesystem>

//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Real-Time Headers
#include <cadmium/basic_model/pdevs/generator.hpp>
#include <cadmium/modeling/coupling.hpp>
#include <cadmium/concept/coupled_model_assert.hpp>
//#include <cadmium/engine/pdevs_runner.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "../include/message_structures/message_mavlink_mission_item_t.hpp"
#include "../include/message_structures/message_fcc_command_t.hpp"

// Project information headers this is created by cmake at generation time!!!!
#include "../include/SupervisorConfig.hpp"
#include "../include/input_readers.hpp" // Input Reader Definitions.

//Coupled model headers
#include "../include/coupled_models/Supervisor.hpp"

using namespace std;
using namespace cadmium;
using namespace cadmium::embedded;
//using namespace cadmium::basic_models::pdevs;

using hclock = chrono::high_resolution_clock;
using TIME = NDTime;

// Used for oss_sink_state and oss_sink_messages
ofstream out_messages;
ofstream out_state;
ofstream out_info;

// Define output ports to be used for logging purposes
struct o_LP_expired : public out_port<message_mavlink_mission_item_t> {};
struct o_start_LZE_scan : public out_port<bool> {};
struct o_mission_complete : public out_port<bool> {};
struct o_land_requested : public out_port<bool> {};
struct o_fcc_command_velocity : public out_port<message_fcc_command_t> {};
struct o_control_yielded : public out_port<bool> {};
struct o_notify_pilot : public out_port<bool> {};
struct o_fcc_command_hover : public out_port<message_fcc_command_t> {};


int main(int argc, char* argv[]) {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/rt_supervisor/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/rt_supervisor/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_landing_achieved = input_dir + string("/landing_achieved.txt");
		string input_file_aircraft_state = input_dir + string("/aircraft_state.txt");
		string input_file_LP_criteria_met = input_dir + string("/LP_criteria_met.txt");
		string input_file_pilot_takeover = input_dir + string("/pilot_takeover.txt");
		string input_file_LP_recv = input_dir + string("/LP_recv.txt");
		string input_file_PLP_ach = input_dir + string("/PLP_ach.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");
		string out_info_file = out_directory + string("/output_info.txt");

		if (!filesystem::exists(input_file_landing_achieved) ||
			!filesystem::exists(input_file_aircraft_state) ||
			!filesystem::exists(input_file_LP_criteria_met) ||
			!filesystem::exists(input_file_pilot_takeover) ||
			!filesystem::exists(input_file_LP_recv) ||
			!filesystem::exists(input_file_PLP_ach)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the coupled model to test
		shared_ptr<dynamic::modeling::coupled<TIME>> supervisor = make_shared<dynamic::modeling::coupled<TIME>>("supervisor", submodels_Supervisor, iports_Supervisor, oports_Supervisor, eics_Supervisor, eocs_Supervisor, ics_Supervisor);

		// Instantiate the input readers.
		// One for each input
		shared_ptr<dynamic::modeling::model> ir_landing_achieved =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_landing_achieved", move(input_file_landing_achieved.c_str()));
		shared_ptr<dynamic::modeling::model> ir_aircraft_state =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Aircraft_State, TIME, const char* >("ir_aircraft_state", move(input_file_aircraft_state.c_str()));
		shared_ptr<dynamic::modeling::model> ir_lp_criteria_met =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_lp_criteria_met", move(input_file_LP_criteria_met.c_str()));
		shared_ptr<dynamic::modeling::model> ir_pilot_takeover =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_pilot_takeover", move(input_file_pilot_takeover.c_str()));
		shared_ptr<dynamic::modeling::model> ir_lp_recv =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_lp_recv", move(input_file_LP_recv.c_str()));
		shared_ptr<dynamic::modeling::model> ir_plp_ach =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_plp_ach", move(input_file_PLP_ach.c_str()));

		// The models to be included in this coupled model 
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
			supervisor,
			ir_landing_achieved,
			ir_aircraft_state,
			ir_lp_criteria_met,
			ir_pilot_takeover,
			ir_lp_recv,
			ir_plp_ach
		};

		dynamic::modeling::Ports iports_TestDriver = {	};

		dynamic::modeling::Ports oports_TestDriver = {
			typeid(o_LP_expired),
			typeid(o_start_LZE_scan),
			typeid(o_mission_complete),
			typeid(o_land_requested),
			typeid(o_fcc_command_velocity),
			typeid(o_control_yielded),
			typeid(o_notify_pilot),
			typeid(o_fcc_command_hover)
		};

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = {
			dynamic::translate::make_EOC<Supervisor_defs::o_LP_expired, o_LP_expired>("supervisor"),
			dynamic::translate::make_EOC<Supervisor_defs::o_start_LZE_scan, o_start_LZE_scan>("supervisor"),
			dynamic::translate::make_EOC<Supervisor_defs::o_mission_complete, o_mission_complete>("supervisor"),
			dynamic::translate::make_EOC<Supervisor_defs::o_land_requested, o_land_requested>("supervisor"),
			dynamic::translate::make_EOC<Supervisor_defs::o_fcc_command_velocity, o_fcc_command_velocity>("supervisor"),
			dynamic::translate::make_EOC<Supervisor_defs::o_control_yielded, o_control_yielded>("supervisor"),
			dynamic::translate::make_EOC<Supervisor_defs::o_notify_pilot, o_notify_pilot>("supervisor"),
			dynamic::translate::make_EOC<Supervisor_defs::o_fcc_command_hover, o_fcc_command_hover>("supervisor")
		};

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
			dynamic::translate::make_IC<Input_Reader_Boolean_defs::out, Supervisor_defs::i_landing_achieved>("ir_landing_achieved", "supervisor"),
			dynamic::translate::make_IC<Input_Reader_Aircraft_State_defs::out, Supervisor_defs::i_aircraft_state>("ir_aircraft_state", "supervisor"),
			dynamic::translate::make_IC<Input_Reader_Mavlink_Mission_Item_defs::out, Supervisor_defs::i_LP_criteria_met>("ir_lp_criteria_met", "supervisor"),
			dynamic::translate::make_IC<Input_Reader_Boolean_defs::out, Supervisor_defs::i_pilot_takeover>("ir_pilot_takeover", "supervisor"),
			dynamic::translate::make_IC<Input_Reader_Mavlink_Mission_Item_defs::out, Supervisor_defs::i_LP_recv>("ir_lp_recv", "supervisor"),
			dynamic::translate::make_IC<Input_Reader_Mavlink_Mission_Item_defs::out, Supervisor_defs::i_PLP_ach>("ir_plp_ach", "supervisor")
		};

		shared_ptr<dynamic::modeling::coupled<TIME>> test_driver = make_shared<dynamic::modeling::coupled<TIME>>(
			"test_driver", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
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

		out_info = ofstream(out_info_file);
		struct oss_sink_info {
			static ostream& sink() {
				return out_info;
			}
		};

		using state = logger::logger<logger::logger_state, dynamic::logger::formatter<TIME>, oss_sink_state>;
		using log_messages = logger::logger<logger::logger_messages, dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_mes = logger::logger<logger::logger_global_time, dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_sta = logger::logger<logger::logger_global_time, dynamic::logger::formatter<TIME>, oss_sink_state>;
		using info = logger::logger<logger::logger_info, dynamic::logger::formatter<TIME>, oss_sink_info>;
		using logger_supervisor = logger::multilogger<state, log_messages, global_time_mes, global_time_sta, info>;

		auto start = hclock::now(); //to measure simulation execution time

		cadmium::dynamic::engine::runner<NDTime, logger_supervisor> r(test_driver, { TIME("00:00:00:000:000") });
		r.run_until(TIME("00:01:00:000:000"));

		auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>
			(hclock::now() - start).count();
		cout << "Simulation took: " << elapsed << " seconds" << endl;

		test_set_enumeration++;
	} while (filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));
	return 0;
}
