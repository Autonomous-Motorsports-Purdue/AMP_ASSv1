/*
 * serialization.cpp
 * Used for accessing functions used in amp_serial_jetson files
 */

#include "serial.h"

/*
 * FUNCTION: 
 * 
 * amp_err_code_t amp_serial_jetson_initialize(sp_port * _port)
 *
 * initializes any given port
 */
 amp_err_code_t serial::amp_serial_jetson_initialize() {
	// Declare & Initialize Local Variables

	#ifdef DEBUG
	fprintf(fptr1, "Initializing Serial Protocol...\n");
	fprintf(fptr1, "Looking for port %s.\n", port_name);
	#endif    
	// Get the Port Handle by the Passed Name
	check(sp_get_port_by_name(port_name, &port), AMP_SERIAL_ERROR_INIT);

	// Display some basic information about the port.
	#ifdef DEBUG
	fprintf(fptr1, "Port name: %s\n", sp_get_port_name(port));
	fprintf(fptr1, "Description: %s\n", sp_get_port_description(port));

	// Open the Serial Port based on the Previous Handle
	fprintf(fptr1, "Opening port...\n");
	#endif
	check(sp_open(port, SP_MODE_READ_WRITE), AMP_SERIAL_ERROR_INIT);

	#ifdef DEBUG
	fprintf(fptr1, "Setting configurations...\n");
	#endif
	amp_serial_jetson_config_port(port, config);

	#ifdef DEBUG    
	fprintf(fptr1, "Checking configurations...\n");
	#endif
	amp_serial_jetson_check_port(port, config);

	return AMP_ERROR_NONE;
}

/*
 * FUNCTION:
 * 
 * void amp_serial_jetson_config_port(sp_port * _port, sp_port_config _config)
 *
 * writes the configurations for any given port
 */
void serial::amp_serial_jetson_config_port(sp_port * _port, sp_port_config _config) {
	
	// Set the Configuration Parameters
	_config.baudrate   =  AMP_SERIAL_CONFIG_BAUD;
	_config.bits       =  AMP_SERIAL_CONFIG_BITS;
	_config.parity     =  AMP_SERIAL_CONFIG_PARY;
	_config.stopbits   =  AMP_SERIAL_CONFIG_STOP;
	_config.cts        =  AMP_SERIAL_CONFIG_CTS;
	_config.dsr        =  AMP_SERIAL_CONFIG_DSR;
	_config.dtr        =  AMP_SERIAL_CONFIG_DTR;
	_config.rts        =  AMP_SERIAL_CONFIG_RTS;
	_config.xon_xoff   =  AMP_SERIAL_CONFIG_XST;

	check(sp_set_config(_port, &_config), AMP_SERIAL_ERROR_INIT);
	
}

/*
 * FUNCTION:
 * 
 * vvoid amp_serial_jetson_check_port(sp_port * _port, sp_port_config _config)
 *
 * checks the configurations for any given port
 */
void serial::amp_serial_jetson_check_port(sp_port * _port, sp_port_config _config) {
	
	int baudrate;
	int bits;
	int stopbits;
	struct sp_port_config * check_config;
	enum sp_parity parity;
	
	check(sp_new_config(&check_config), AMP_SERIAL_ERROR_INIT);
	check(sp_get_config(_port, check_config), AMP_SERIAL_ERROR_INIT);
	
	check(sp_get_config_baudrate(check_config, &baudrate), AMP_SERIAL_ERROR_INIT);
	check(sp_get_config_bits(check_config, &bits), AMP_SERIAL_ERROR_INIT);
	check(sp_get_config_stopbits(check_config, &stopbits), AMP_SERIAL_ERROR_INIT);
	check(sp_get_config_parity(check_config, &parity), AMP_SERIAL_ERROR_INIT);
	sp_free_config(check_config);

	#ifdef DEBUG
	fprintf(fptr1, "Baudrate: %d, data bits: %d, parity: %s, stop bits: %d\n", baudrate, bits, parity_name(parity), stopbits);
	fprintf(fptr1, "Successfully opened port (%s) at %d\n", port_name , baudrate);

	if(_config.baudrate != baudrate)
	{
	fprintf(fptr1, "Baudrate mismatch\n");
	}
	if(_config.bits != bits)
	{
	fprintf(fptr1, "Bits mismatch\n");
	}
	if(_config.parity != parity)
	{
	fprintf(fptr1, "Parity mismatch\n");
	}
	if(_config.stopbits != stopbits)
	{
	fprintf(fptr1, "Stopbit mismatch\n");
	}
	#endif
}

// HELPER FUNCTIONS

/*
 * FUNCTION:
 *
 * const char *parity_name(enum sp_parity parity)
 * 
 * returns the parity state of the buffer
 */
const char* serial::parity_name(enum sp_parity parity)
{
	switch (parity) {
	case SP_PARITY_INVALID:
		return "(Invalid)";
	case SP_PARITY_NONE:
		return "None";
	case SP_PARITY_ODD:
		return "Odd";
	case SP_PARITY_EVEN:
		return "Even";
	case SP_PARITY_MARK:
		return "Mark";
	case SP_PARITY_SPACE:
		return "Space";
	default:
		return NULL;
	}
}

/*
 * FUNCTION:
 *
 * void end_program(amp_err_code_t amp_err)
 * 
 * clean termination program for handling serial comm errors
 */
void serial::end_program(amp_err_code_t amp_err)
{
	/* Free any structures we allocated. */
	if (&config != NULL)
		sp_free_config(&config);
	if (port != NULL)
		sp_free_port(port);

	/* Exit with the given return code. */
	exit(amp_err);
}

/*
 * FUNCTION:
 *
 * int check(enum sp_return result, amp_err_code_t amp_err)
 * 
 * error handling function for serial communication
 */
int serial::check(enum sp_return result, amp_err_code_t amp_err)
{
	int error_code;
	char *error_message;

	switch (result) {

	case SP_ERR_ARG:
		/* When SP_ERR_ARG is returned, there was a problem with one
		 * or more of the arguments passed to the function, e.g. a null
		 * pointer or an invalid value. This generally implies a bug in
		 * the calling code. */
		printf("Error: Invalid argument.\n");
		#ifdef DEBUG
		fprintf(fptr1, "Error: Invalid argument.\n");
		#endif
		end_program(amp_err);

	case SP_ERR_FAIL:
		/* When SP_ERR_FAIL is returned, there was an error from the OS,
		 * which we can obtain the error code and message for. These
		 * calls must be made in the same thread as the call that
		 * returned SP_ERR_FAIL, and before any other system functions
		 * are called in that thread, or they may not return the
		 * correct results. */
		error_code = sp_last_error_code();
		error_message = sp_last_error_message();
		printf("Error: Failed: OS error code: %d, message: '%s'\n", error_code, error_message);
		#ifdef DEBUG
		fprintf(fptr1, "Error: Failed: OS error code: %d, message: '%s'\n", error_code, error_message);
		#endif
		/* The error message should be freed after use. */
		sp_free_error_message(error_message);
		end_program(amp_err);

	case SP_ERR_SUPP:
		/* When SP_ERR_SUPP is returned, the function was asked to do
		 * something that isn't supported by the current OS or device,
		 * or that libserialport doesn't know how to do in the current
		 * version. */
		printf("Error: Not supported.\n");
		#ifdef DEBUG
		fprintf(fptr1, "Error: Not supported.\n");
		#endif
		end_program(amp_err);

	case SP_ERR_MEM:
		/* When SP_ERR_MEM is returned, libserialport wasn't able to
		 * allocate some memory it needed. Since the library doesn't
		 * normally use any large data structures, this probably means
		 * the system is critically low on memory and recovery will
		 * require very careful handling. The library itself will
		 * always try to handle any allocation failure safely. */
		end_program(amp_err);

	case SP_OK:
	default:
		#ifdef DEBUG
		fprintf(fptr1, "Operation succeeded.\n");
		#endif
		return result;
	}
}

void serial::amp_serial_jetson_enable_kart() {
	// Declare & Initialize Local Variables
	amp_serial_pkt_t t_pkt;
	int size;

	#ifdef DEBUG
	fprintf(fptr1, "Enabling Kart through Packet ID\n");
	#endif

	// Enable the kart
	t_pkt.id = AMP_SERIAL_ENABLE;
	t_pkt.size = 0;
	size = 0;

	amp_serial_jetson_tx_pkt(&t_pkt, &size);
}

void serial::amp_serial_jetson_enable_drive() {
	// Declare & Initialize Local Variables
	amp_serial_pkt_t t_pkt;
	int size;

	#ifdef DEBUG
	fprintf(fptr1, "Putting Kart in Drive Mode\n");
	#endif

	t_pkt.id = AMP_SERIAL_DRIVE;
	t_pkt.size = 0;
	size = 0;

	amp_serial_jetson_tx_pkt(&t_pkt, &size);
}

void serial::amp_serial_jetson_enable_default() {
	// Declare & Initialize Local Variables
	amp_serial_pkt_t t_pkt;
	int size;

	#ifdef DEBUG
	fprintf(fptr1, "Putting Kart in Default Mode\n");
	#endif

	t_pkt.id = AMP_SERIAL_DEFAULT;
	t_pkt.size = 0;
	size = 0;

	amp_serial_jetson_tx_pkt(&t_pkt, &size);
}

/*
 * FUNCTION:
 *
 * void argparse(int &argc, char *argv[], string &filename, float &delay) {
 * 
 * Parse input args
 */
void serial::argparse(int &argc, char *argv[], string &filename, float &delay) {
	vector<string> args(argv + 1, argv + argc);
	for (auto i = args.begin(); i != args.end(); ++i) {
		if (*i == "-h" || *i == "--help") {
			cout << "Help: " << endl;
			cout << "-f or --file : Read commands from specified filename." << endl;
			cout << "               File must have speed and velocity separated by a space, each "  
				"command being on a new line" << endl;
			cout << "               If no filename is specified will try to read from 'debug.txt'" << endl;
			cout << "               Example file: \n"
				"               5.4 2.4\n"
				"               2.5 3.9" << endl;
			cout << "-d or --delay : Set tx and rx delay" << endl;
			cout << "-h or --help : Display this help menu" << endl;
			exit(0);
		} else if (*i == "-f" || *i == "--file") {
			filename = *++i;
		} else if (*i == "-d" || *i == "--delay") {
			delay = stof(*++i);
		}
	}
}

