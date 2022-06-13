#include <string>

#include <boost/asio.hpp>

#include <cadmium/modeling/dynamic_model.hpp>

using namespace std;
using namespace cadmium::dynamic::modeling;

template <typename MSG>
class UDP_Callback {
private:
	mutable std::vector<MSG> message;
	// asio::ip::udp::endpoint network_endpoint;
	// asio::ip::udp::endpoint remote_endpoint;
	// asio::io_service io_service;
	// asio::ip::udp::socket socket{ io_service };
	// bool send_ack;
	// char recv_buffer[MAX_SER_BUFFER_CHARS];


public:
	UDP_Callback(bool ack_required, string address, string port) {
		// send_ack = ack_required;
		// unsigned short port_num = (unsigned short)strtoul(port.c_str(), NULL, 0);
		// network_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(address), port_num);
	}

	void monitor(AsyncEventSubject* sub) {

	}

	int read() {

	}
};