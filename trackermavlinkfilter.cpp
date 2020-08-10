/****************************************************************************
 *
 *   Copyright (c) 2020 unmanned-technologies.de. All rights reserved.
 *   Author:  Stefan Arbes <info@unmanned-technologies.de>
 *
 *   based partly on the work of the MAVlink Development Team.
 *           Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *           Hannes Diethelm, <hannes.diethelm@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name unmanned-technologies.de nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file trackermavlinkfilter.cpp
 *
 * @brief Mavlink-Filter for MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_MSG_ID_ATTITUDE
 *
 * Streams an external UDP MavLink stream to a MAVLink UART device, but only tracker relevant mavlink messages mentioned above
 * Accepts Mavlink2 as input, outputs Mavlink1 for reasons of compatibility
 * Input: stdin or UDP, Output UART
 *
 * @author Stefan Arbes <info@unmanned-technologies.de>
 *
 */

#include "trackermavlinkfilter.h"

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int top (int argc, char **argv)
{
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 115200;

	char *udp_ip = (char*)"127.0.0.1";
	int udp_port = 14552;

	parse_commandline(argc, argv, uart_name, baudrate, udp_ip, udp_port);

	Generic_Port *source_port;
	Generic_Port *dest_port;
	if ( udp_port != -1 ) {
		source_port = new UDP_Port(udp_ip, udp_port);
	} else {
		source_port = new STDIO_Port();
	}
	dest_port = new Serial_Port(uart_name, baudrate);

	TrackerFilter_Interface trackerfilter_interface(source_port, dest_port);

	source_port_quit         = source_port;
	dest_port_quit           = dest_port;

	trackerfilter_interface_quit = &trackerfilter_interface;
	signal(SIGINT,quit_handler);

	source_port->start();
	dest_port->start();
	trackerfilter_interface.start();

	while (1) 
	{
		sleep (60000);
	}

	trackerfilter_interface.stop();
	source_port->stop();

	delete source_port;
	delete dest_port;

	return 0;

}

void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, char *&udp_ip, int &udp_port)
{

	const char *commandline_usage = "usage: trackermavlinkfilter -d <devicename> -b <baudrate> -u <udp_ip> -p <udp_port> -n (-n means use of STDIN instead of UDP)";

	for (int i = 1; i < argc; i++) 
	{ 
		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				i++;
				uart_name = argv[i];
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				i++;
				baudrate = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP ip
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--udp_ip") == 0) {
			if (argc > i + 1) {
				i++;
				udp_ip = argv[i];
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP port
		if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
			if (argc > i + 1) {
				i++;
				udp_port = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// STDIO input
		if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--stdio") == 0) {
			udp_port = -1;
		}
	}
	return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
void quit_handler( int sig )
{
	fprintf(stderr,"\n");
	fprintf(stderr,"TERMINATING AT USER REQUEST\n");
	fprintf(stderr,"\n");
	try {
		trackerfilter_interface_quit->handle_quit(sig);
	}
	catch (int error){}
	try {
		source_port_quit->stop();
	}
	catch (int error){}
	try {
		dest_port_quit->stop();
	}
	catch (int error){}
	exit(0);
}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		int result = top(argc,argv);
		return result;
	}
	catch ( int error )
	{
		fprintf(stderr,"trackermavlinkfilter threw exception %i \n" , error);
		return error;
	}
}


