//C++ headers
#include <chrono>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>

//Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

// Project information headers this is created by cmake at generation time!!!!
#include "SupervisorConfig.hpp"
#include "io_models/Supervisor_UDP_Input.hpp"
#include "io_models/Aircraft_State_Input.hpp"
#include "io_models/Polling_Condition_Input.hpp"
#include "io_models/Packet_Builder.hpp"
#include "io_models/UDP_Output.hpp"
#include "io_models/RUDP_Output.hpp"
#include "io_models/GPS_Time.h"

//Coupled model headers
#include "coupled_models/Supervisor.hpp"

using namespace cadmium;

using hclock = std::chrono::high_resolution_clock;
using TIME = NDTime;

int main() {
	const string out_directory = string(PROJECT_DIRECTORY) + string("/test/simulation_results/supervisor/0");

	string out_messages_file = out_directory + string("/output_messages.txt");
	string out_state_file = out_directory + string("/output_state.txt");
	string out_info_file = out_directory + string("/output_info.txt");

	// Create the output location
	boost::filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

	// Instantiate the coupled model
	Supervisor supervisor_instance = Supervisor();
	shared_ptr<dynamic::modeling::coupled<TIME>> supervisor = make_shared<dynamic::modeling::coupled<TIME>>("supervisor", supervisor_instance.submodels, supervisor_instance.iports, supervisor_instance.oports, supervisor_instance.eics, supervisor_instance.eocs, supervisor_instance.ics);

	// Instantiate the input readers.
	shared_ptr<dynamic::modeling::model> im_udp_interface = dynamic::translate::make_dynamic_atomic_model<Supervisor_UDP_Input, TIME, TIME, unsigned short>("im_udp_interface", std::move(TIME("00:00:00:100")), 23001);
	shared_ptr<dynamic::modeling::model> im_aircraft_state = dynamic::translate::make_dynamic_atomic_model<Aircraft_State_Input, TIME>("im_aircraft_state");
	shared_ptr<dynamic::modeling::model> im_landing_achieved = dynamic::translate::make_dynamic_atomic_model<Polling_Condition_Input_Landing_Achieved, TIME, TIME, float>("im_landing_achieved", std::move(TIME("00:00:00:100")), DEFAULT_LAND_CRITERIA_VERT_DIST);
	shared_ptr<dynamic::modeling::model> im_pilot_takeover = dynamic::translate::make_dynamic_atomic_model<Polling_Condition_Input_Pilot_Takeover, TIME, TIME>("im_pilot_takeover", std::move(TIME("00:00:01:000")));

    // Instantiate the Packet Builders.
    shared_ptr<dynamic::modeling::model> pb_bool_mission_complete = dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Bool, TIME, uint8_t>("pb_bool_mission_complete", SIG_ID_MISSION_COMPLETE);
    shared_ptr<dynamic::modeling::model> pb_int_mission_start = dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Int, TIME, uint8_t>("pb_int_mission_start", SIG_ID_START_MISSION);
    shared_ptr<dynamic::modeling::model> pb_bool_update_mission_item = dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Bool, TIME, uint8_t>("pb_bool_update_mission_item", SIG_ID_MISSION_ITEM_REACHED);
    shared_ptr<dynamic::modeling::model> pb_uint8_set_mission_monitor_status = dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Uint8, TIME, uint8_t>("pb_uint8_set_mission_monitor_status", SIG_ID_SET_MISSION_MONITOR_STATUS);

    shared_ptr<dynamic::modeling::model> pb_boss = dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Boss, TIME>("pb_boss");
    shared_ptr<dynamic::modeling::model> pb_fcc = dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Fcc, TIME>("pb_fcc");
    shared_ptr<dynamic::modeling::model> pb_gcs = dynamic::translate::make_dynamic_atomic_model<Packet_Builder_GCS, TIME>("pb_gcs");
    shared_ptr<dynamic::modeling::model> pb_landing_point = dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Landing_Point, TIME>("pb_landing_point");

    shared_ptr<dynamic::modeling::model> udp_boss = dynamic::translate::make_dynamic_atomic_model<UDP_Output, TIME, const char *, const unsigned short>("udp_boss", IPV4_BOSS, PORT_BOSS);
    shared_ptr<dynamic::modeling::model> udp_fcc = dynamic::translate::make_dynamic_atomic_model<UDP_Output, TIME, const char *, const unsigned short>("udp_fcc", IPV4_FCC, PORT_FCC);
    shared_ptr<dynamic::modeling::model> udp_gcs = dynamic::translate::make_dynamic_atomic_model<UDP_Output, TIME, const char *, const unsigned short>("udp_gcs", IPV4_GCS, PORT_GCS);
    shared_ptr<dynamic::modeling::model> rudp_mavnrc = dynamic::translate::make_dynamic_atomic_model<RUDP_Output, TIME, const char *, const unsigned short, int, int>("rudp_mavnrc", IPV4_MAVNRC, PORT_MAVNRC, DEFAULT_TIMEOUT_MS, 10);

    // Instantiate GPS time logger
    shared_ptr<dynamic::modeling::model> gps_time = dynamic::translate::make_dynamic_atomic_model<GPS_Time, TIME>("a_gps_time");

    // The models to be included in this coupled model
	// (accepts atomic and coupled models)
	dynamic::modeling::Models submodels_TestDriver = {
            supervisor,
            im_landing_achieved,
            im_aircraft_state,
            im_pilot_takeover,
            im_udp_interface,
            pb_bool_mission_complete,
            pb_int_mission_start,
			pb_bool_update_mission_item,
            pb_uint8_set_mission_monitor_status,
            pb_boss,
            pb_fcc,
            pb_gcs,
            pb_landing_point,
            udp_boss,
            udp_fcc,
            udp_gcs,
            gps_time,
            rudp_mavnrc
	};

	dynamic::modeling::Ports iports_TestDriver = { };

	dynamic::modeling::Ports oports_TestDriver = { };

	dynamic::modeling::EICs eics_TestDriver = { };

	// The output ports will be used to export in logging
	dynamic::modeling::EOCs eocs_TestDriver = {	};

	// This will connect our outputs from our input reader to the file
	dynamic::modeling::ICs ics_TestDriver = {
		dynamic::translate::make_IC<Polling_Condition_Input_Landing_Achieved<TIME>::defs::o_message, Supervisor_defs::i_landing_achieved>("im_landing_achieved", "supervisor"),
		dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_land, Polling_Condition_Input_Landing_Achieved<TIME>::defs::i_start>("supervisor", "im_landing_achieved"),
		dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Polling_Condition_Input_Landing_Achieved<TIME>::defs::i_quit>("supervisor", "im_landing_achieved"),

		dynamic::translate::make_IC<Aircraft_State_Input_defs::o_message, Supervisor_defs::i_aircraft_state>("im_aircraft_state", "supervisor"),
		dynamic::translate::make_IC<Supervisor_defs::o_request_aircraft_state, Aircraft_State_Input_defs::i_request>("supervisor", "im_aircraft_state"),

		dynamic::translate::make_IC<Polling_Condition_Input_Pilot_Takeover<TIME>::defs::o_message, Supervisor_defs::i_pilot_takeover>("im_pilot_takeover", "supervisor"),
		// dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_start_supervisor, Polling_Condition_Input_Pilot_Takeover<TIME>::defs::i_start>("im_udp_interface", "im_pilot_takeover"),
		// dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Polling_Condition_Input_Pilot_Takeover<TIME>::defs::i_quit>("supervisor", "im_pilot_takeover"),

		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_lp_recv, Supervisor_defs::i_LP_recv>("im_udp_interface", "supervisor"),
		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_plp_ach, Supervisor_defs::i_PLP_ach>("im_udp_interface", "supervisor"),
		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_perception_status, Supervisor_defs::i_perception_status>("im_udp_interface", "supervisor"),
		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_start_supervisor, Supervisor_defs::i_start_supervisor>("im_udp_interface", "supervisor"),
		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_waypoint, Supervisor_defs::i_waypoint>("im_udp_interface", "supervisor"),
		// dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Supervisor_UDP_Input_defs::i_quit>("supervisor", "im_udp_interface"),

		// Output ICs
        dynamic::translate::make_IC<Supervisor_defs::o_LP_new, Packet_Builder_Landing_Point<TIME>::defs::i_data>("supervisor", "pb_landing_point"),

        dynamic::translate::make_IC<Supervisor_defs::o_start_mission, Packet_Builder_Int<TIME>::defs::i_data>("supervisor", "pb_int_mission_start"),
        dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Packet_Builder_Bool<TIME>::defs::i_data>("supervisor", "pb_bool_mission_complete"),
        dynamic::translate::make_IC<Supervisor_defs::o_update_mission_item, Packet_Builder_Bool<TIME>::defs::i_data>("supervisor", "pb_bool_update_mission_item"),
        dynamic::translate::make_IC<Supervisor_defs::o_set_mission_monitor_status, Packet_Builder_Uint8<TIME>::defs::i_data>("supervisor", "pb_uint8_set_mission_monitor_status"),

        dynamic::translate::make_IC<Supervisor_defs::o_update_boss, Packet_Builder_Boss<TIME>::defs::i_data>("supervisor", "pb_boss"),
        dynamic::translate::make_IC<Supervisor_defs::o_update_gcs, Packet_Builder_GCS<TIME>::defs::i_data>("supervisor", "pb_gcs"),

        dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_hover, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),
        dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_land, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),
        dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_orbit, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),
        dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_velocity, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),
        dynamic::translate::make_IC<Supervisor_defs::o_fcc_waypoint_update, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),

        dynamic::translate::make_IC<Packet_Builder_Boss<TIME>::defs::o_packet, UDP_Output<TIME>::defs::i_message>("pb_boss", "udp_boss"),
        dynamic::translate::make_IC<Packet_Builder_Fcc<TIME>::defs::o_packet, UDP_Output<TIME>::defs::i_message>("pb_fcc", "udp_fcc"),
        dynamic::translate::make_IC<Packet_Builder_GCS<TIME>::defs::o_packet, UDP_Output<TIME>::defs::i_message>("pb_gcs", "udp_gcs"),

        dynamic::translate::make_IC<Packet_Builder_Int<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_int_mission_start", "rudp_mavnrc"),
        dynamic::translate::make_IC<Packet_Builder_Bool<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_bool_mission_complete", "rudp_mavnrc"),
        dynamic::translate::make_IC<Packet_Builder_Bool<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_bool_update_mission_item", "rudp_mavnrc"),
        dynamic::translate::make_IC<Packet_Builder_Uint8<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_uint8_set_mission_monitor_status", "rudp_mavnrc"),
        dynamic::translate::make_IC<Packet_Builder_Landing_Point<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_landing_point", "rudp_mavnrc"),
	};

	shared_ptr<dynamic::modeling::coupled<TIME>> test_driver = make_shared<dynamic::modeling::coupled<TIME>>(
		"test_driver", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
	);

	/*************** Loggers *******************/
    static ofstream out_messages;
    static ofstream out_state;
    static ofstream out_info;

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
	r.run_until_passivate();

	auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(hclock::now() - start).count();
	cout << "Simulation took: " << elapsed << " seconds" << endl;

	fflush(nullptr);
	string path_to_script = PROJECT_DIRECTORY + string("/test/scripts/simulation_cleanup.py");
	string path_to_simulation_results = PROJECT_DIRECTORY + string("/test/simulation_results");
	if (std::system("python3 --version") == 0) {
		string command = "python3 " + path_to_script + string(" ") + path_to_simulation_results;
		std::system(command.c_str());
	} else if (std::system("python --version") == 0) {
		string command = "python " + path_to_script + string(" ") + path_to_simulation_results;
		std::system(command.c_str());
	} else {
		cout << "\nPython is not installed!\n";
	}

	return 0;
}
