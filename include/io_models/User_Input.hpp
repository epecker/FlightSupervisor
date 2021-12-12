/**
 *	\brief		An atomic model for receiving command line input from the user.
 *	\details	This header file defines an asynchronous input atomic model 
                for use in the RT-Cadmium DEVS simulation software.
 *	\author		James Horner
 */

#ifndef USER_INPUT_HPP
#define USER_INPUT_HPP

#include <iostream>

#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

using namespace cadmium;
using namespace std;

//Port definition
struct User_Input_defs{
    struct in   : public in_port<bool> { };
    struct out  : public out_port<bool> { };
};

template<typename TIME>
class User_Input {
    private:
        cadmium::dynamic::modeling::AsyncEventSubject *_sub;

    public:

    enum States {
        IDLE,
        INPUT
    };

    // default constructor
    User_Input() {
        state.current_state = States::IDLE;
    }

    // state definition
    struct state_type{
        States current_state;
        bool input_received;
    };
    state_type state;

    // ports definition
    using input_ports=std::tuple<typename User_Input_defs::in>;
    using output_ports=std::tuple<typename User_Input_defs::out>;

    // internal transition
    void internal_transition() {
        if (state.current_state == States::INPUT) {
            state.current_state = States::IDLE;
        }
    }

    // external transition
    void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
        bool input_received = get_messages<typename User_Input_defs::in>(mbs).size() >= 1;
        if (state.current_state == States::IDLE && input_received) {
            state.current_state = States::INPUT;
            string s;
            cout << "Please enter any input:\t";
            cin >> s;
            state.input_received = true;
        }
    }

    // confluence transition
    void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
        internal_transition();
        external_transition(TIME(), std::move(mbs));
    }

    // output function
    typename make_message_bags<output_ports>::type output() const {
        typename make_message_bags<output_ports>::type bags;
        if(state.current_state == States::INPUT && state.input_received) {
            get_messages<typename User_Input_defs::out>(bags).push_back(true);
        }
        return bags;
    }

    // time_advance function
    TIME time_advance() const {
        if(state.current_state == States::IDLE){
            return std::numeric_limits<TIME>::infinity();
        }

        return TIME("00:00:00:00");
    }

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename User_Input<TIME>::state_type& i) {
        os << "State: " << i.current_state;
        return os;
    }
};

#endif /* USER_INPUT_HPP */
