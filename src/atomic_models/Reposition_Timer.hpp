/**
 * 	\file		Reposition_Timer.hpp
 *	\brief		Definition of the Reposition Timer atomic model.
 *	\details	This header file defines the Reposition Timer atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when valid LPs
                are found and must be repositioned to within a specified amount of time.
 *	\image		html atomic_models/reposition_timer.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef REPOSITION_TIMER_HPP
#define REPOSITION_TIMER_HPP

// Messages structures
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_boss_mission_update_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

// Utility functions
#include "../time_conversion.hpp"
#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// System Libraries
#include <limits> // Used to set the time advance to infinity
#include <cassert>
#include <string>

/**
 * 	\class		Reposition_Timer
 *	\brief		Definition of the Reposition Timer atomic model.
 *	\details	This class defines the Reposition Timer atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when valid LPs
                are found and must be repositioned to within a specified amount of time.
 *	\image		html atomic_models/reposition_timer.png
 */
template<typename TIME>
class Reposition_Timer {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
    DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
        (IDLE)
        (WAIT_NEW_LP)
        (NOTIFY_UPDATE)
        (UPDATE_LP)
        (LP_REPO)
        (NEW_LP_REPO)
        (REQUEST_LAND)
        (HANDOVER_CTRL)
        (LANDING_ROUTINE)
        (PILOT_CONTROL)
    );

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Reposition_Timer_input_ports "Input Ports" and
	 *	\ref 	Reposition_Timer_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
    struct defs {
        struct i_control_yielded : public cadmium::in_port<bool> {};
        struct i_lp_crit_met : public cadmium::in_port<message_landing_point_t> {};
        struct i_lp_new : public cadmium::in_port<message_landing_point_t> {};
        struct i_pilot_takeover : public cadmium::in_port<bool> {};
        struct i_start_mission : public cadmium::in_port<int> {};

        struct o_cancel_hover : public cadmium::out_port<bool> {};
        struct o_land : public cadmium::out_port<message_landing_point_t> {};
        struct o_pilot_handover : public cadmium::out_port<message_landing_point_t> {};
        struct o_request_reposition : public cadmium::out_port<message_landing_point_t> {};
        struct o_update_boss : public cadmium::out_port<message_boss_mission_update_t> {};
        struct o_update_gcs : public cadmium::out_port<message_update_gcs_t> {};
    };

	/**
	 * 	\anchor	Reposition_Timer_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param	i_control_yielded	[input] Port for receiving signal indicating control has been handed over to the pilot
     *  \param  i_lp_crit_met       [input] Port for receiving signal indicating that the helicopter is now hovering over a landing point.
     *  \param  i_lp_new            [input] Port for receiving new valid landing points that should be repositioned to.
	 * 	\param 	i_pilot_takeover 	[input] Port for receiving signal indicating that the pilot has taken control from the supervisor.
	 * 	\param 	i_start_mission 	[input] Port for receiving signal indicating the mission has started.
     */
    using input_ports = std::tuple<
            typename defs::i_control_yielded,
            typename defs::i_lp_crit_met,
            typename defs::i_lp_new,
            typename defs::i_pilot_takeover,
            typename defs::i_start_mission
    >;

	/**
	 *	\anchor	Reposition_Timer_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
     *  \param  o_land                  Port for requesting that the landing be attempted at a landing point.
	 * 	\param	o_cancel_hover 			Port for cancelling a previously requested stabilization.
	 * 	\param	o_pilot_handover		Port for requesting that control be handed over to the pilot.
     *  \param  o_request_reposition    Port for requesting that a landing point be repositioned to.
	 * 	\param	o_update_boss 			Port for sending updates to BOSS.
	 * 	\param	o_update_gcs 			Port for sending updates to the GCS.
     */
    using output_ports = std::tuple<
            typename defs::o_land,
            typename defs::o_cancel_hover,
            typename defs::o_pilot_handover,
            typename defs::o_request_reposition,
            typename defs::o_update_boss,
            typename defs::o_update_gcs
    >;

	/**
	 *	\anchor	Reposition_Timer_state_type
	 *	\par	State
	 * 	Definition of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 */
    struct state_type {
        States current_state;
    } state;

	/**
	 * \brief 	Default constructor for the model.
	 */
    Reposition_Timer() {
        state.current_state = States::IDLE;
        mission_number = 0;
        repo_time = seconds_to_time<TIME>(REPO_TIMER);
		upd_time = seconds_to_time<TIME>(UPD_TIMER);
        landing_point = message_landing_point_t();
        last_lp = 0;
    }

	/**
	 * \brief 	Constructor for the model with parameters for the length of the reposition and update timers.
	 * \param	i_lp_accept_time	TIME length of the reposition timer.
	 * \param	i_orbit_time		TIME length of the update timer.
	 */
    explicit Reposition_Timer(TIME i_repo_time, TIME i_upd_time) {
        state.current_state = States::IDLE;
        mission_number = 0;
        repo_time = i_repo_time;
        upd_time = i_upd_time;
        landing_point = message_landing_point_t();
        last_lp = 0;
    }

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	i_lp_accept_time	TIME length of the reposition timer.
	 * \param	i_orbit_time		TIME length of the update timer.
	 * \param	initial_state	    States initial state of the model.
	 */
    Reposition_Timer(TIME i_repo_time, TIME i_upd_time, States initial_state) {
        state.current_state = initial_state;
        mission_number = 0;
        repo_time = i_repo_time;
		upd_time = i_upd_time;
        landing_point = message_landing_point_t();
        last_lp = 0;
    }

	/// Internal transitions of the model
    void internal_transition() {
        switch (state.current_state) {
            case States::NOTIFY_UPDATE:
                state.current_state = States::UPDATE_LP;
                break;
            case States::UPDATE_LP:
                state.current_state = States::NEW_LP_REPO;
                break;
            case States::NEW_LP_REPO:
                state.current_state = States::LP_REPO;
                break;
            case States::LP_REPO:
                state.current_state = States::HANDOVER_CTRL;
                break;
            case States::REQUEST_LAND:
                state.current_state = States::LANDING_ROUTINE;
                break;
            default:
                break;
        }
    }

	/// External transitions of the model
    void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
        bool received_pilot_takeover = !cadmium::get_messages<typename defs::i_pilot_takeover>(mbs).empty();
        if (received_pilot_takeover) {
            state.current_state = States::PILOT_CONTROL;
            return;
        }

        bool received_start_mission = !cadmium::get_messages<typename Reposition_Timer<TIME>::defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            reset_state();
            mission_number = cadmium::get_messages<typename Reposition_Timer<TIME>::defs::i_start_mission>(mbs).back();
            state.current_state = States::WAIT_NEW_LP;
            return;
        }

        switch (state.current_state) {
            case States::WAIT_NEW_LP: {
				bool received_lp_new = !cadmium::get_messages<typename defs::i_lp_new>(mbs).empty();
				if (received_lp_new) {
					std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename defs::i_lp_new>(
							mbs);
					// Get the most recent landing point input (found at the back of the vector of inputs)
					landing_point = new_landing_points.back();
					state.current_state = States::NOTIFY_UPDATE;
				}
				break;
			}
            case States::UPDATE_LP: {
				bool received_lp_new = !cadmium::get_messages<typename defs::i_lp_new>(mbs).empty();
				if (received_lp_new) {
					std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename defs::i_lp_new>(mbs);
					// Get the most recent landing point input (found at the back of the vector of inputs)
					landing_point = new_landing_points.back();
					update_upd_time(e);
					state.current_state = States::NOTIFY_UPDATE;
				}
				break;
			}
            case States::LP_REPO: {
				bool received_lp_new = !cadmium::get_messages<typename defs::i_lp_new>(mbs).empty();
				bool received_lp_crit_met = !cadmium::get_messages<typename defs::i_lp_crit_met>(mbs).empty();
				if (received_lp_new) {
					std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename defs::i_lp_new>(mbs);
					// Get the most recent landing point input (found at the back of the vector of inputs)
					landing_point = new_landing_points.back();
					state.current_state = States::NEW_LP_REPO;
				} else if (received_lp_crit_met) {
					state.current_state = States::REQUEST_LAND;
				}
				break;
			}
            case States::HANDOVER_CTRL: {
				bool received_control_yielded = !cadmium::get_messages<typename defs::i_control_yielded>(mbs).empty();
				if (received_control_yielded) {
					state.current_state = States::PILOT_CONTROL;
				}
				break;
			}
            default:
                break;
        }
    }

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
    void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
        internal_transition();
        external_transition(TIME(), std::move(mbs));
    }

	/// Function for generating output from the model before internal transitions.
    [[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
        typename cadmium::make_message_bags<output_ports>::type bags;

        switch (state.current_state) {
			case States::NOTIFY_UPDATE: {
				if (last_lp == 0) {
					// Update the ground control computer
					cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
							"LP found. Holding for " + std::to_string(upd_time.getSeconds()) + "s",
							Mav_Severities_E::MAV_SEVERITY_ALERT
					);
				}

				if (landing_point.id != last_lp) {
					last_lp = landing_point.id;
					// Update the boss displays landing point location
					cadmium::get_messages<typename defs::o_update_boss>(bags).emplace_back(
							landing_point.id,
							landing_point.lat,
							landing_point.lon,
							mission_number,
							landing_point.missionItemNo,
							landing_point.alt * FT_TO_METERS,
							landing_point.hdg,
							0,
							"LP UPD"
					);
				}
				break;
			}
            case States::REQUEST_LAND: {
                cadmium::get_messages<typename defs::o_land>(bags).push_back(landing_point);
                break;
            }
            case States::LP_REPO: {
				// Update BOSS display message
				cadmium::get_messages<typename defs::o_update_boss>(bags).emplace_back(
						mission_number,
						landing_point.alt * FT_TO_METERS,
						"LZ SCAN"
				);

				// Update the ground control computer
				cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
						"Repo timer expired, hovering over the last LP",
						Mav_Severities_E::MAV_SEVERITY_ALERT
				);

                cadmium::get_messages<typename defs::o_cancel_hover>(bags).emplace_back(true);
                cadmium::get_messages<typename defs::o_pilot_handover>(bags).push_back(landing_point);
                break;
            }
            case States::NEW_LP_REPO:
                cadmium::get_messages<typename defs::o_request_reposition>(bags).push_back(landing_point);
                break;
            default:
                break;
        }

        return bags;
    }

	/// Function to declare the time advance value for each state of the model.
    TIME time_advance() const {
        TIME next_internal;

        switch(state.current_state) {
            case States::IDLE:
            case States::WAIT_NEW_LP:
            case States::HANDOVER_CTRL:
            case States::PILOT_CONTROL:
            case States::LANDING_ROUTINE:
                next_internal = std::numeric_limits<TIME>::infinity();
                break;
            case States::UPDATE_LP:
                next_internal = upd_time;
                break;
            case States::LP_REPO:
                next_internal = repo_time;
                break;
            case States::NOTIFY_UPDATE:
            case States::NEW_LP_REPO:
            case States::REQUEST_LAND:
                next_internal = TIME(TA_ZERO);
                break;
            default:
                assert(false && "Unhandled state time advance.");
        }

        return next_internal;
    }

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
    friend std::ostringstream& operator<<(std::ostringstream& os, const typename Reposition_Timer<TIME>::state_type& i) {
        os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
        return os;
    }

private:
    /// Variable for storing the current valid landing point being repositioned to.
    message_landing_point_t landing_point;
    /// Variable for storing the number of the mission for updating BOSS.
    int mission_number;
    /// Variable for storing the length of the reposition timer.
    TIME repo_time;
    /// Variable for storing the remaining length of the update timer.
	TIME upd_time;
    /// Variable for storing the number of the last landing point, to check if one has been received yet.
    mutable int last_lp;

    /// Function for resetting private variables.
    void reset_state() {
        mission_number = 0;
        repo_time = TIME(LP_REPOSITION_TIME);
        landing_point = message_landing_point_t();
		upd_time = seconds_to_time<TIME>(UPD_TIMER);
        last_lp = 0;
    }

    /**
     * \brief   Function update_upd_time is used to update the update timer based on the current state and an elapsed time.
     */
	void update_upd_time(TIME e) {
        upd_time = upd_time - e;
        if (upd_time <= TIME(TA_ZERO)) {
            upd_time = TIME(TA_ZERO);
        }
    }
};

#endif // REPOSITION_TIMER_HPP
