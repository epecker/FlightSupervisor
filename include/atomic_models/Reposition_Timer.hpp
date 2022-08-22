/**
 *	\brief		An atomic model representing the reposition model.
 *	\details	This header file define the reposition model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef REPOSITION_TIMER_HPP
#define REPOSITION_TIMER_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <cassert> // Used to check values and stop the simulation
#include <string>

// Includes the macro DEFINE_ENUM_WITH_STRING_CONVERSIONS
#include "enum_string_conversion.hpp"

// Data structures that are used in message transport
#include "message_structures/message_landing_point_t.hpp"
#include "message_structures/message_boss_mission_update_t.hpp"
#include "message_structures/message_update_gcs_t.hpp"

// Macros
#include "Constants.hpp"

using namespace cadmium;

// Atomic Model
template<typename TIME> class Reposition_Timer {
public:
    // Used to keep track of the states
    // (not required for the simulator)
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

    // Input and output port definition
    struct defs {
        struct i_control_yielded : public in_port<bool> {};
        struct i_lp_crit_met : public in_port<message_landing_point_t> {};
        struct i_lp_new : public in_port<message_landing_point_t> {};
        struct i_pilot_takeover : public in_port<bool> {};
        struct i_start_mission : public in_port<int> {};

        struct o_cancel_hover : public out_port<bool> {};
        struct o_land : public out_port<message_landing_point_t> {};
        struct o_pilot_handover : public out_port<message_landing_point_t> {};
        struct o_request_reposition : public out_port<message_landing_point_t> {};
        struct o_update_boss : public out_port<message_boss_mission_update_t> {};
        struct o_update_gcs : public out_port<message_update_gcs_t> {};
    };

    // Create a tuple of input ports (required for the simulator)
    using input_ports = tuple<
            typename Reposition_Timer::defs::i_control_yielded,
            typename Reposition_Timer::defs::i_lp_crit_met,
            typename Reposition_Timer::defs::i_lp_new,
            typename Reposition_Timer::defs::i_pilot_takeover,
            typename Reposition_Timer::defs::i_start_mission
    >;

    // Create a tuple of output ports (required for the simulator)
    using output_ports = tuple<
            typename Reposition_Timer::defs::o_land,
            typename Reposition_Timer::defs::o_cancel_hover,
            typename Reposition_Timer::defs::o_pilot_handover,
            typename Reposition_Timer::defs::o_request_reposition,
            typename Reposition_Timer::defs::o_update_boss,
            typename Reposition_Timer::defs::o_update_gcs
    >;

    // This is used to track the state of the atomic model.
    // (required for the simulator)
    struct state_type {
        States current_state;
    } state;

    // Default constructor
    Reposition_Timer() {
        state.current_state = States::IDLE;
        mission_number = 0;
        repo_time = seconds_to_time<TIME>(REPO_TIMER);
		upd_time = seconds_to_time<TIME>(UPD_TIMER);
        landing_point = message_landing_point_t();
        last_lp = 0;
    }

    // Constructor with timer parameter
    explicit Reposition_Timer(TIME i_repo_time, TIME i_upd_time) {
        state.current_state = States::IDLE;
        mission_number = 0;
        repo_time = i_repo_time;
        upd_time = i_upd_time;
        landing_point = message_landing_point_t();
        last_lp = 0;
    }

    // Constructor with timer parameter and initial state parameter for debugging or partial execution startup.
    Reposition_Timer(TIME i_repo_time, TIME i_upd_time, States initial_state) {
        state.current_state = initial_state;
        mission_number = 0;
        repo_time = i_repo_time;
		upd_time = i_upd_time;
        landing_point = message_landing_point_t();
        last_lp = 0;
    }

    // Internal transitions
    // These are transitions occurring from internal inputs
    // (required for the simulator)
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

    // External transitions
    // These are transitions occurring from external inputs
    // (required for the simulator)
    void external_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
        bool received_control_yielded;
        bool received_lp_new;
        bool received_lp_crit_met;

        bool received_pilot_takeover = !get_messages<typename Reposition_Timer::defs::i_pilot_takeover>(mbs).empty();
        if (received_pilot_takeover) {
            state.current_state = States::PILOT_CONTROL;
            return;
        }

        bool received_start_mission = !get_messages<typename Reposition_Timer<TIME>::defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            reset_state();
            mission_number = get_messages<typename Reposition_Timer<TIME>::defs::i_start_mission>(mbs).back();
            state.current_state = States::WAIT_NEW_LP;
            return;
        }

        switch (state.current_state) {
            case States::WAIT_NEW_LP:
                received_lp_new = !get_messages<typename Reposition_Timer::defs::i_lp_new>(mbs).empty();
                if (received_lp_new) {
                    vector<message_landing_point_t> new_landing_points = get_messages<typename Reposition_Timer::defs::i_lp_new>(mbs);
                    // Get the most recent landing point input (found at the back of the vector of inputs)
                    landing_point = new_landing_points.back();
                    state.current_state = States::NOTIFY_UPDATE;
                }
                break;
            case States::UPDATE_LP:
                received_lp_new = !get_messages<typename Reposition_Timer::defs::i_lp_new>(mbs).empty();
                if (received_lp_new) {
                    vector<message_landing_point_t> new_landing_points = get_messages<typename Reposition_Timer::defs::i_lp_new>(mbs);
                    // Get the most recent landing point input (found at the back of the vector of inputs)
                    landing_point = new_landing_points.back();
                    update_upd_time(e);
                    state.current_state = States::NOTIFY_UPDATE;
                }
                break;
            case States::LP_REPO:
                received_lp_new = !get_messages<typename Reposition_Timer::defs::i_lp_new>(mbs).empty();
                received_lp_crit_met = !get_messages<typename Reposition_Timer::defs::i_lp_crit_met>(mbs).empty();

                if (received_lp_new) {
                    vector<message_landing_point_t> new_landing_points = get_messages<typename Reposition_Timer::defs::i_lp_new>(mbs);
                    // Get the most recent landing point input (found at the back of the vector of inputs)
                    landing_point = new_landing_points.back();
                    state.current_state = States::NEW_LP_REPO;
                } else if (received_lp_crit_met) {
                    state.current_state = States::REQUEST_LAND;
                }
                break;
            case States::HANDOVER_CTRL:
                received_control_yielded = !get_messages<typename Reposition_Timer::defs::i_control_yielded>(mbs).empty();

                if (received_control_yielded) {
                    state.current_state = States::PILOT_CONTROL;
                }
                break;
            default:
                break;
        }
    }

    // confluence transition
    // Used to call set call precedent
    void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
        internal_transition();
        external_transition(TIME(), std::move(mbs));
    }

    // output function
    [[nodiscard]] typename make_message_bags<output_ports>::type output() const {
        typename make_message_bags<output_ports>::type bags;
        vector<bool> bag_port_out;
        vector<message_landing_point_t> bag_port_lp_out;
        vector<message_boss_mission_update_t> boss_messages;
        vector<message_update_gcs_t> gcs_messages;

        switch (state.current_state) {
            case States::NOTIFY_UPDATE: {
                if (last_lp == 0) {
                    std::string message = "LP found. Holding for " + std::to_string(upd_time.getSeconds()) + "s";
                    message_update_gcs_t temp_gcs_update{message, Mav_Severities_E::MAV_SEVERITY_ALERT};
                    get_messages<typename Reposition_Timer::defs::o_update_gcs>(bags).push_back(temp_gcs_update);
                }

                if (landing_point.id != last_lp) {
                    message_boss_mission_update_t temp_boss{};
                    temp_boss.update_landing_point(
                            landing_point.id,
                            landing_point.lat,
                            landing_point.lon,
                            landing_point.alt,
                            landing_point.hdg,
                            "LP UPD");
                    temp_boss.missionNo = mission_number;
                    get_messages<typename Reposition_Timer::defs::o_update_boss>(bags).push_back(temp_boss);
                    last_lp = landing_point.id;
                }

                break;
            }
            case States::REQUEST_LAND: {
                get_messages<typename Reposition_Timer::defs::o_land>(bags).push_back(landing_point);
                break;
            }
            case States::LP_REPO: {
                message_boss_mission_update_t temp_boss{};
                temp_boss.update_message("MAN CTRL", false, mission_number);

                message_update_gcs_t temp_gcs("Repo timer expired, hovering over the last LP", Mav_Severities_E::MAV_SEVERITY_ALERT);
                boss_messages.push_back(temp_boss);
                gcs_messages.push_back(temp_gcs);
                bag_port_lp_out.push_back(landing_point);
                bag_port_out.push_back(true);
                get_messages<typename Reposition_Timer::defs::o_cancel_hover>(bags) = bag_port_out;
                get_messages<typename Reposition_Timer::defs::o_update_boss>(bags) = boss_messages;
                get_messages<typename Reposition_Timer::defs::o_update_gcs>(bags) = gcs_messages;
                get_messages<typename Reposition_Timer::defs::o_pilot_handover>(bags) = bag_port_lp_out;
                break;
            }
            case States::NEW_LP_REPO:
                bag_port_lp_out.push_back(landing_point);
                get_messages<typename Reposition_Timer::defs::o_request_reposition>(bags) = bag_port_lp_out;
                break;
            default:
                break;
        }

        return bags;
    }

    // Time advance
    // Used to set the internal time of the current state
    TIME time_advance() const {
        TIME next_internal;

        switch(state.current_state) {
            case States::IDLE:
            case States::WAIT_NEW_LP:
            case States::HANDOVER_CTRL:
            case States::PILOT_CONTROL:
            case States::LANDING_ROUTINE:
                next_internal = numeric_limits<TIME>::infinity();
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

    friend ostringstream& operator<<(ostringstream& os, const typename Reposition_Timer<TIME>::state_type& i) {
        os << (string("State: ") + enumToString(i.current_state) + string("\n"));
        return os;
    }

private:
    message_landing_point_t landing_point;
    int mission_number;
    TIME repo_time;
	TIME upd_time;
    mutable int last_lp;

    void reset_state() {
        mission_number = 0;
        repo_time = TIME(LP_REPOSITION_TIME);
        landing_point = message_landing_point_t();
		upd_time = seconds_to_time<TIME>(UPD_TIMER);
        last_lp = 0;
    }

	void update_upd_time(TIME e) {
        upd_time = upd_time - e;
        if (upd_time <= TIME(TA_ZERO)) {
            upd_time = TIME(TA_ZERO);
        }
    }

};

#endif // REPOSITION_TIMER_HPP
