//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "../data_structures/message.hpp"

//Atomic model headers
#include <cadmium/basic_model/pdevs/iestream.hpp> //Atomic model for inputs
#include "../atomic_models/Handover_Ctrl.hpp"
#include "../atomic_models/Landing_Routine.hpp"
#include "../atomic_models/LP_Manager.hpp"
#include "../atomic_models/LP_Reposition.hpp"
#include "../atomic_models/Stabilize.hpp"

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

/***** Define input port for coupled models *****/
struct i_lp_criteria_met : public in_port<Message_t> {};
struct i_pilot_takeover : public in_port<Message_t> {};
struct i_lp_recv : public in_port<Message_t> {};
struct i_plp_ach : public in_port<Message_t> {};
struct i_hover_criteria : public in_port<Message_t> {};
struct i_landing_achieved : public in_port<Message_t> {};

/***** Define output ports for coupled model *****/
struct o_control_yielded : public out_port<Message_t> {};
struct o_notify_pilot : public out_port<Message_t> {};
struct o_land_requested : public out_port<Message_t> {};
struct o_mission_complete : public out_port<Message_t> {};


/**
* Create an atomic model to read the input file.
*/
template<typename T>
class InputReader_Message_t : public iestream_input<Message_t, T> {
public:
	InputReader_Message_t() = default;
	InputReader_Message_t(const char* file_path) : iestream_input<Message_t, T>(file_path) {}
};

int main(int argc, char* argv[]) {
	if (argc < 2) {
		printf("The program should be invoked as follows\n");
		printf("%s path/to/input/file\n", argv[0]);
		return 1;
	}

	if (!filesystem::exists(argv[1])) {
		printf("The input file does not exist\n");
		return 1;
	}

	/**
	* Instantiate the InputReader_Int Atomic model.
	*/
	string input = argv[1];
	const char* i_input = input.c_str();
	shared_ptr<dynamic::modeling::model> input_reader = dynamic::translate::make_dynamic_atomic_model<InputReader_Message_t, TIME, const char* >("input_reader", move(i_input));

	/*
	 * ==========================================================
	 * SUPERVISOR COUPLED MODEL
	 * ==========================================================
	 */

	 /**
	 * Instantiate the Man, Repo, and Hand Atomic models.
	 */
	shared_ptr<dynamic::modeling::model> handover_ctrl = dynamic::translate::make_dynamic_atomic_model<Handover_Ctrl, TIME>("handover_ctrl");
	shared_ptr<dynamic::modeling::model> landing_routine = dynamic::translate::make_dynamic_atomic_model<Landing_Routine, TIME>("landing_routine");
	shared_ptr<dynamic::modeling::model> lp_manager = dynamic::translate::make_dynamic_atomic_model<LP_Manager, TIME>("lp_manager");
	shared_ptr<dynamic::modeling::model> lp_reposition = dynamic::translate::make_dynamic_atomic_model<LP_Reposition, TIME>("lp_reposition");
	shared_ptr<dynamic::modeling::model> stabilize = dynamic::translate::make_dynamic_atomic_model<Stabilize, TIME>("stabilize");


	//Define the inputs to the Supervisor coupled model.
	dynamic::modeling::Ports iports_Supervisor = {
		typeid(i_lp_criteria_met),
		typeid(i_pilot_takeover),
		typeid(i_lp_recv),
		typeid(i_plp_ach),
		typeid(i_hover_criteria),
		typeid(i_landing_achieved)
	};

	//Define the outputs of the Supervisor coupled model.
	dynamic::modeling::Ports oports_Supervisor = {
		typeid(o_control_yielded),
		typeid(o_notify_pilot),
		typeid(o_land_requested),
		typeid(o_mission_complete)
	};

	//Define the sub-models that make up the Supervisor coupled model.
	dynamic::modeling::Models submodels_Supervisor = {
		handover_ctrl,
		landing_routine,
		lp_manager,
		lp_reposition,
		stabilize
	};

	//Define the external to internal couplings for the Supervisor.
	dynamic::modeling::EICs eics_Supervisor = {
		// handover_ctrl
		dynamic::translate::make_EIC<i_pilot_takeover, Handover_Ctrl_defs::i_pilot_takeover>("handover_ctrl"),
		// landing_routine
		dynamic::translate::make_EIC<i_pilot_takeover, Landing_Routine_defs::i_pilot_takeover>("landing_routine"),
		dynamic::translate::make_EIC<i_landing_achieved, Landing_Routine_defs::i_landing_achieved>("landing_routine"),
		// lp_manager
		dynamic::translate::make_EIC<i_lp_recv, LP_Manager_defs::i_lp_recv>("lp_manager"),
		dynamic::translate::make_EIC<i_plp_ach, LP_Manager_defs::i_plp_ach>("lp_manager"),
		dynamic::translate::make_EIC<i_pilot_takeover, LP_Manager_defs::i_pilot_takeover>("lp_manager"),
		// lp_reposition
		dynamic::translate::make_EIC<i_lp_criteria_met, LP_Reposition_defs::i_lp_crit_met>("lp_reposition"),
		dynamic::translate::make_EIC<i_pilot_takeover, LP_Reposition_defs::i_pilot_takeover>("lp_reposition"),
		// stabilize
		dynamic::translate::make_EIC<i_hover_criteria, Stabilize_defs::i_hover_criteria>("stabilize")
	};

	//Define the internal to external couplings for the Supervisor.
	dynamic::modeling::EOCs eocs_Supervisor = {
		// handover_ctrl
		dynamic::translate::make_EOC<Handover_Ctrl_defs::o_control_yielded, o_control_yielded>("handover_ctrl"),
		dynamic::translate::make_EOC<Handover_Ctrl_defs::o_notify_pilot, o_notify_pilot>("handover_ctrl"),
		// landing_routine
		dynamic::translate::make_EOC<Landing_Routine_defs::o_land_requested, o_land_requested>("landing_routine"),
		dynamic::translate::make_EOC<Landing_Routine_defs::o_land_requested, o_land_requested>("landing_routine")
		// lp_manager
		// lp_reposition
		// stabilize
	};

	//Define the internal to internal couplings for the Supervisor.
	dynamic::modeling::ICs ics_Supervisor = {
		// handover_ctrl
		// dynamic::translate::make_IC<Handover_Ctrl_defs::o_control_yielded, LP_Manager_defs::>("handover_ctrl","lp_manager"), // TODO
		// dynamic::translate::make_IC<Handover_Ctrl_defs::o_control_yielded, LP_Reposition_defs::i_control_yielded>("handover_ctrl","lp_reposition"),
		// landing_routine
		// lp_manager
		// dynamic::translate::make_IC<LP_Manager_defs::o_control_yielded, LP_Manager_defs::>("handover_ctrl","lp_manager"), // TODO
		// lp_reposition

		// stabilize
	};

	//Declare an object for the Supervisor.
	shared_ptr<dynamic::modeling::coupled<TIME>> SUPERVISOR;
	//Instantiate the Supervisor with the ports, models, and couplings previsouly defined.
	SUPERVISOR = make_shared<dynamic::modeling::coupled<TIME>>(
		"Supervisor", submodels_Supervisor, iports_Supervisor, oports_Supervisor, eics_Supervisor, eocs_Supervisor, ics_Supervisor
	);


	/*
	 * ==========================================================
	 * TEST DRIVER COUPLED MODEL
	 * ==========================================================
	 */

	dynamic::modeling::Ports iports_TestDriver = {	};

	dynamic::modeling::Ports oports_TestDriver = {	};

	dynamic::modeling::Models submodels_TestDriver = {
		input_reader,
		SUPERVISOR
	};

	dynamic::modeling::EICs eics_TestDriver = {	};
	dynamic::modeling::EOCs eocs_TestDriver = {	};

	//Define the one coupling from the input reader to the landing point receipt port on the Supervisor.
	dynamic::modeling::ICs ics_TestDriver = {
		dynamic::translate::make_IC<iestream_input_defs<Message_t>::out, i_lp_criteria_met>("input_reader", "Supervisor")
	};

	shared_ptr<dynamic::modeling::coupled<TIME>> TEST_DRIVER;
	TEST_DRIVER = make_shared<dynamic::modeling::coupled<TIME>>(
		"TestDriver", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
	);

	/*************** Loggers *******************/
	string out_directory = string(PROJECT_DIRECTORY) + string("/simulation_results");
	string out_messages_file = string(PROJECT_DIRECTORY) + string("/simulation_results/output_messages.txt");
	string out_state_file = string(PROJECT_DIRECTORY) + string("/simulation_results/output_state.txt");

	filesystem::create_directory(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

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