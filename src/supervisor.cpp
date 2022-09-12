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



using hclock = std::chrono::high_resolution_clock;
using TIME = NDTime;

int main() {
	const std::string out_directory = std::string(PROJECT_DIRECTORY) + std::string("/test/simulation_results/supervisor/0");

	std::string out_messages_file = out_directory + std::string("/output_messages.txt");
	std::string out_state_file = out_directory + std::string("/output_state.txt");
	std::string out_info_file = out_directory + std::string("/output_info.txt");

	// Create the output location
	boost::filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

	// Instantiate the coupled model
	Supervisor supervisor_instance = Supervisor();
	std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> supervisor = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>("supervisor", supervisor_instance.submodels, supervisor_instance.iports, supervisor_instance.oports, supervisor_instance.eics, supervisor_instance.eocs, supervisor_instance.ics);

	// Instantiate the input readers.
	std::shared_ptr<cadmium::dynamic::modeling::model> im_udp_interface = cadmium::dynamic::translate::make_dynamic_atomic_model<Supervisor_UDP_Input, TIME, TIME, unsigned short>("im_udp_interface", std::move(TIME("00:00:00:100")), 23001);
	std::shared_ptr<cadmium::dynamic::modeling::model> im_aircraft_state = cadmium::dynamic::translate::make_dynamic_atomic_model<Aircraft_State_Input, TIME>("im_aircraft_state");
	std::shared_ptr<cadmium::dynamic::modeling::model> im_landing_achieved = cadmium::dynamic::translate::make_dynamic_atomic_model<Polling_Condition_Input_Landing_Achieved, TIME, TIME, float>("im_landing_achieved", std::move(TIME("00:00:00:100")), DEFAULT_LAND_CRITERIA_VERT_DIST);
	std::shared_ptr<cadmium::dynamic::modeling::model> im_pilot_takeover = cadmium::dynamic::translate::make_dynamic_atomic_model<Polling_Condition_Input_Pilot_Takeover, TIME, TIME>("im_pilot_takeover", std::move(TIME("00:00:01:000")));

    // Instantiate the Packet Builders.
	std::shared_ptr<cadmium::dynamic::modeling::model> pb_bool_mission_complete = cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Bool, TIME, uint8_t>("pb_bool_mission_complete", SIG_ID_MISSION_COMPLETE);
    std::shared_ptr<cadmium::dynamic::modeling::model> pb_int_mission_start = cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Int, TIME, uint8_t>("pb_int_mission_start", SIG_ID_START_MISSION);
    std::shared_ptr<cadmium::dynamic::modeling::model> pb_bool_update_mission_item = cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Bool, TIME, uint8_t>("pb_bool_update_mission_item", SIG_ID_MISSION_ITEM_REACHED);
    std::shared_ptr<cadmium::dynamic::modeling::model> pb_uint8_set_mission_monitor_status = cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Uint8, TIME, uint8_t>("pb_uint8_set_mission_monitor_status", SIG_ID_SET_MISSION_MONITOR_STATUS);

    std::shared_ptr<cadmium::dynamic::modeling::model> pb_boss = cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Boss, TIME>("pb_boss");
    std::shared_ptr<cadmium::dynamic::modeling::model> pb_fcc = cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Fcc, TIME>("pb_fcc");
    std::shared_ptr<cadmium::dynamic::modeling::model> pb_gcs = cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_GCS, TIME>("pb_gcs");
    std::shared_ptr<cadmium::dynamic::modeling::model> pb_landing_point = cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Landing_Point, TIME>("pb_landing_point");

    std::shared_ptr<cadmium::dynamic::modeling::model> udp_boss = cadmium::dynamic::translate::make_dynamic_atomic_model<UDP_Output, TIME, const char *, const unsigned short, bool>("udp_boss", IPV4_BOSS, PORT_BOSS, true);
    std::shared_ptr<cadmium::dynamic::modeling::model> udp_fcc = cadmium::dynamic::translate::make_dynamic_atomic_model<UDP_Output, TIME, const char *, const unsigned short, bool>("udp_fcc", IPV4_FCC, PORT_FCC, true);
    std::shared_ptr<cadmium::dynamic::modeling::model> udp_gcs = cadmium::dynamic::translate::make_dynamic_atomic_model<UDP_Output, TIME, const char *, const unsigned short, bool>("udp_gcs", IPV4_GCS, PORT_GCS, false);
    std::shared_ptr<cadmium::dynamic::modeling::model> udp_gcs_broadcast = cadmium::dynamic::translate::make_dynamic_atomic_model<UDP_Output, TIME, const char *, const unsigned short, bool>("udp_gcs_broadcast", IPV4_QGC_BROADCAST, PORT_QGC_BROADCAST, true);
    std::shared_ptr<cadmium::dynamic::modeling::model> rudp_mavnrc = cadmium::dynamic::translate::make_dynamic_atomic_model<RUDP_Output, TIME, const char *, const unsigned short, int, int>("rudp_mavnrc", IPV4_MAVNRC, PORT_MAVNRC, DEFAULT_TIMEOUT_MS, 10);

    // Instantiate GPS time logger
	std::shared_ptr<cadmium::dynamic::modeling::model> gps_time = cadmium::dynamic::translate::make_dynamic_atomic_model<GPS_Time, TIME>("a_gps_time");

    // The models to be included in this coupled model
	// (accepts atomic and coupled models)
 	cadmium::dynamic::modeling::Models submodels_TestDriver = {
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
			udp_gcs_broadcast,
            gps_time,
            rudp_mavnrc
	};

 	cadmium::dynamic::modeling::Ports iports_TestDriver = { };

 	cadmium::dynamic::modeling::Ports oports_TestDriver = { };

 	cadmium::dynamic::modeling::EICs eics_TestDriver = { };

	// The output ports will be used to export in logging
 	cadmium::dynamic::modeling::EOCs eocs_TestDriver = {	};

	// This will connect our outputs from our input reader to the file
 	cadmium::dynamic::modeling::ICs ics_TestDriver = {
		cadmium::dynamic::translate::make_IC<Polling_Condition_Input_Landing_Achieved<TIME>::defs::o_message, Supervisor_defs::i_landing_achieved>("im_landing_achieved", "supervisor"),
		cadmium::dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_land, Polling_Condition_Input_Landing_Achieved<TIME>::defs::i_start>("supervisor", "im_landing_achieved"),
		cadmium::dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Polling_Condition_Input_Landing_Achieved<TIME>::defs::i_quit>("supervisor", "im_landing_achieved"),

		cadmium::dynamic::translate::make_IC<Aircraft_State_Input_defs::o_message, Supervisor_defs::i_aircraft_state>("im_aircraft_state", "supervisor"),
		cadmium::dynamic::translate::make_IC<Supervisor_defs::o_request_aircraft_state, Aircraft_State_Input_defs::i_request>("supervisor", "im_aircraft_state"),

		cadmium::dynamic::translate::make_IC<Polling_Condition_Input_Pilot_Takeover<TIME>::defs::o_message, Supervisor_defs::i_pilot_takeover>("im_pilot_takeover", "supervisor"),
		// cadmium::dynamic::translate::make_IC<Supervisor_UDP_Input<TIME>::defs::o_start_supervisor, Polling_Condition_Input_Pilot_Takeover<TIME>::defs::i_start>("im_udp_interface", "im_pilot_takeover"),
		// cadmium::dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Polling_Condition_Input_Pilot_Takeover<TIME>::defs::i_quit>("supervisor", "im_pilot_takeover"),

		cadmium::dynamic::translate::make_IC<Supervisor_UDP_Input<TIME>::defs::o_lp_recv, Supervisor_defs::i_LP_recv>("im_udp_interface", "supervisor"),
		cadmium::dynamic::translate::make_IC<Supervisor_UDP_Input<TIME>::defs::o_plp_ach, Supervisor_defs::i_PLP_ach>("im_udp_interface", "supervisor"),
		cadmium::dynamic::translate::make_IC<Supervisor_UDP_Input<TIME>::defs::o_perception_status, Supervisor_defs::i_perception_status>("im_udp_interface", "supervisor"),
		cadmium::dynamic::translate::make_IC<Supervisor_UDP_Input<TIME>::defs::o_start_supervisor, Supervisor_defs::i_start_supervisor>("im_udp_interface", "supervisor"),
		cadmium::dynamic::translate::make_IC<Supervisor_UDP_Input<TIME>::defs::o_waypoint, Supervisor_defs::i_waypoint>("im_udp_interface", "supervisor"),
		// cadmium::dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Supervisor_UDP_Input<TIME>::defs::i_quit>("supervisor", "im_udp_interface"),

		// Output ICs
        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_LP_new, Packet_Builder_Landing_Point<TIME>::defs::i_data>("supervisor", "pb_landing_point"),

        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_start_mission, Packet_Builder_Int<TIME>::defs::i_data>("supervisor", "pb_int_mission_start"),
        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Packet_Builder_Bool<TIME>::defs::i_data>("supervisor", "pb_bool_mission_complete"),
        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_update_mission_item, Packet_Builder_Bool<TIME>::defs::i_data>("supervisor", "pb_bool_update_mission_item"),
        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_set_mission_monitor_status, Packet_Builder_Uint8<TIME>::defs::i_data>("supervisor", "pb_uint8_set_mission_monitor_status"),

        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_update_boss, Packet_Builder_Boss<TIME>::defs::i_data>("supervisor", "pb_boss"),
        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_update_gcs, Packet_Builder_GCS<TIME>::defs::i_data>("supervisor", "pb_gcs"),

        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_hover, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),
        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_land, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),
        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_orbit, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),
        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_fcc_command_velocity, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),
        cadmium::dynamic::translate::make_IC<Supervisor_defs::o_fcc_waypoint_update, Packet_Builder_Fcc<TIME>::defs::i_data>("supervisor", "pb_fcc"),

        cadmium::dynamic::translate::make_IC<Packet_Builder_Boss<TIME>::defs::o_packet, UDP_Output<TIME>::defs::i_message>("pb_boss", "udp_boss"),
        cadmium::dynamic::translate::make_IC<Packet_Builder_Fcc<TIME>::defs::o_packet, UDP_Output<TIME>::defs::i_message>("pb_fcc", "udp_fcc"),
        cadmium::dynamic::translate::make_IC<Packet_Builder_GCS<TIME>::defs::o_packet, UDP_Output<TIME>::defs::i_message>("pb_gcs", "udp_gcs"),
        cadmium::dynamic::translate::make_IC<Packet_Builder_GCS<TIME>::defs::o_packet, UDP_Output<TIME>::defs::i_message>("pb_gcs", "udp_gcs_broadcast"),

        cadmium::dynamic::translate::make_IC<Packet_Builder_Int<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_int_mission_start", "rudp_mavnrc"),
        cadmium::dynamic::translate::make_IC<Packet_Builder_Bool<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_bool_mission_complete", "rudp_mavnrc"),
        cadmium::dynamic::translate::make_IC<Packet_Builder_Bool<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_bool_update_mission_item", "rudp_mavnrc"),
        cadmium::dynamic::translate::make_IC<Packet_Builder_Uint8<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_uint8_set_mission_monitor_status", "rudp_mavnrc"),
        cadmium::dynamic::translate::make_IC<Packet_Builder_Landing_Point<TIME>::defs::o_packet, RUDP_Output<TIME>::defs::i_message>("pb_landing_point", "rudp_mavnrc"),
	};

	std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> test_driver = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
		"test_driver", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
	);

	/*************** Loggers *******************/
    static std::ofstream out_messages;
    static std::ofstream out_state;
    static std::ofstream out_info;

	out_messages = std::ofstream(out_messages_file);
	struct oss_sink_messages {
		static std::ostream& sink() {
			return out_messages;
		}
	};

	out_state = std::ofstream(out_state_file);
	struct oss_sink_state {
		static std::ostream& sink() {
			return out_state;
		}
	};

	out_info = std::ofstream(out_info_file);
	struct oss_sink_info {
		static std::ostream& sink() {
			return out_info;
		}
	};

	using state = cadmium::logger::logger<cadmium::logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;
	using log_messages = cadmium::logger::logger<cadmium::logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
	using global_time_mes = cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
	using global_time_sta = cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;
	using info = cadmium::logger::logger<cadmium::logger::logger_info, cadmium::dynamic::logger::formatter<TIME>, oss_sink_info>;
	using logger_supervisor = cadmium::logger::multilogger<state, log_messages, global_time_mes, global_time_sta, info>;

	auto start = hclock::now(); //to measure simulation execution time

	cadmium::dynamic::engine::runner<NDTime, logger_supervisor> r(test_driver, { TIME("00:00:00:000:000") });
	r.run_until_passivate();

	auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(hclock::now() - start).count();
	std::cout << "Simulation took: " << elapsed << " seconds" << std::endl;

	fflush(nullptr);
	std::string path_to_script = PROJECT_DIRECTORY + std::string("/test/scripts/simulation_cleanup.py");
	std::string path_to_simulation_results = PROJECT_DIRECTORY + std::string("/test/simulation_results");
	if (std::system("python3 --version") == 0) {
		std::string command = "python3 " + path_to_script + std::string(" ") + path_to_simulation_results;
		std::system(command.c_str());
	} else if (std::system("python --version") == 0) {
		std::string command = "python " + path_to_script + std::string(" ") + path_to_simulation_results;
		std::system(command.c_str());
	} else {
		std::cout << "\nPython is not installed!\n";
	}

	return 0;
}
