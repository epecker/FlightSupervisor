//
// Created by trautrim on 27/08/22.
//

#ifndef GPS_TIME_H
#define GPS_TIME_H

#include <limits>
#include <ostream>

#include "cadmium/modeling/message_bag.hpp"
#include "sharedmemorymodel/SharedMemoryModel.h"
#include "../enum_string_conversion.hpp"

template<typename TIME>
class GPS_Time {
public:
    DEFINE_ENUM_WITH_STRING_CONVERSIONS(States, (GPS_TIME));

    // Create a tuple of input ports (required for the simulator)
    using input_ports = std::tuple<>;

    // Create a tuple of output ports (required for the simulator)
    using output_ports = std::tuple<>;

    // Tracks the state of the model
    struct state_type {
        States current_state;
    } state;

    GPS_Time() {
        state.current_state = States::GPS_TIME;
        model.connectSharedMem();
        if (!model.isConnected()) {
            throw(std::runtime_error("Could not connect to shared memory."));
        }
    }

    // Destructor for disconnecting from shared memory.
    ~GPS_Time() {
        model.disconnectSharedMem();
    }

    // Internal transitions (required for the simulator)
    void internal_transition() {}

    // External transitions (required for the simulator)
    void external_transition([[maybe_unused]] TIME e, [[maybe_unused]] typename cadmium::make_message_bags<input_ports>::type mbs) {}

    // Confluence transition sets the internal/external precedence
    // Triggered when a message is received at the same time as an internal transition.
    // (required for the simulator)
    void confluence_transition([[maybe_unused]] TIME e, [[maybe_unused]] typename cadmium::make_message_bags<input_ports>::type mbs) {}

    // Creates output messages (required for the simulator)
    [[nodiscard]]
    typename cadmium::make_message_bags<output_ports>::type output() const {
        return {};
    }

    // Time advance sets the wait time of the current state (required for the simulator)
    TIME time_advance() const {
        return std::numeric_limits<TIME>::infinity();
    }

    // Used for logging outputs the state's name. (required for the simulator)
    friend std::ostringstream& operator<<(std::ostringstream& os, const typename GPS_Time<TIME>::state_type& i) {
        os << (std::string("State: ") + std::to_string(model.sharedMemoryStruct->hg1700.time));
        return os;
    }

private:
    inline static SharedMemoryModel model{};

};

#endif // GPS_TIME_H
