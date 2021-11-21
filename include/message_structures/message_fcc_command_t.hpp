#ifndef FCC_COMMAND_HPP
#define FCC_COMMAND_HPP

struct message_fcc_command_t {
	double  temp;		            // Decimal degrees

	message_fcc_command_t() : temp(0){}
};

#endif