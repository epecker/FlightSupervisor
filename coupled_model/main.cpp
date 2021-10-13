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
#include "../atomic_models/Hand.hpp"
#include "../atomic_models/Man.hpp"
#include "../atomic_models/Repo.hpp"

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
struct lp_recv_in : public in_port<Message_t> {};
struct plp_ach_in : public in_port<Message_t> {};
struct pilot_takeover_in : public in_port<Message_t> {};
struct hover_criteria_met_in : public in_port<Message_t> {};
struct lp_crit_met_in : public in_port<Message_t> {};

/***** Define output ports for coupled model *****/
struct repo_new_LP_out : public out_port<Message_t> {};
struct land_out : public out_port<Message_t> {};


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
	shared_ptr<dynamic::modeling::model> hand = dynamic::translate::make_dynamic_atomic_model<Hand, TIME>("hand");
	shared_ptr<dynamic::modeling::model> repo = dynamic::translate::make_dynamic_atomic_model<Repo, TIME>("repo");
	shared_ptr<dynamic::modeling::model> man = dynamic::translate::make_dynamic_atomic_model<Man, TIME>("man");

	//Define the inputs to the Supervisor coupled model.
	dynamic::modeling::Ports iports_Supervisor = { 
		typeid(lp_recv_in),
		typeid(plp_ach_in),
		typeid(pilot_takeover_in),
		typeid(hover_criteria_met_in),
		typeid(lp_crit_met_in)
	};

	//Define the outputs of the Supervisor coupled model.
	dynamic::modeling::Ports oports_Supervisor = { 
		typeid(repo_new_LP_out),
		typeid(land_out)
	};

	//Define the sub-models that make up the Supervisor coupled model.
	dynamic::modeling::Models submodels_Supervisor = { 
		hand,
		repo,
		man
	};

	//Define the external to internal couplings for the Supervisor.
	dynamic::modeling::EICs eics_Supervisor = {
		dynamic::translate::make_EIC<lp_recv_in, Man_defs::lp_recv_in>("man"),
		dynamic::translate::make_EIC<plp_ach_in, Man_defs::plp_ach_in>("man"),
		dynamic::translate::make_EIC<pilot_takeover_in, Hand_defs::pilot_takeover_in>("hand"),
		dynamic::translate::make_EIC<pilot_takeover_in, Man_defs::pilot_takeover_in>("man"),
		dynamic::translate::make_EIC<pilot_takeover_in, Repo_defs::pilot_takeover_in>("repo"),
		dynamic::translate::make_EIC<hover_criteria_met_in, Hand_defs::hover_criteria_met_in>("hand"),
		dynamic::translate::make_EIC<lp_crit_met_in, Repo_defs::lp_crit_met_in>("repo")
	};

	//Define the internal to external couplings for the Supervisor.
	dynamic::modeling::EOCs eocs_Supervisor = {
		dynamic::translate::make_EOC<Repo_defs::lp_repo_new_out, repo_new_LP_out>("repo"),
		dynamic::translate::make_EOC<Repo_defs::land_out, land_out>("repo")
	};

	//Define the internal to internal couplings for the Supervisor.
	dynamic::modeling::ICs ics_Supervisor = {
		dynamic::translate::make_IC<Man_defs::lp_new_out, Repo_defs::lp_new_in>("man","repo"),
		dynamic::translate::make_IC<Man_defs::pilot_handover_out, Hand_defs::pilot_handover_in>("man","hand"),
		dynamic::translate::make_IC<Repo_defs::pilot_handover_out, Hand_defs::pilot_handover_in>("repo","hand")
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
		dynamic::translate::make_IC<iestream_input_defs<Message_t>::out, lp_recv_in>("input_reader", "Supervisor")
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