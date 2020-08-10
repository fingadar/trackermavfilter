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
 * @file serial_port.cpp
 *
 * @brief Serial interface definition
 *
 * Functions for opening, closing, reading and writing via serial ports
 *
 * @author Stefan Arbes <info@unmanned-technologies.de>
 *
 */

#include "serial_port.h"

extern bool debug; 

Serial_Port::Serial_Port(const char *my_uart_name, int my_baudrate)
{
	initialize_defaults();
	uart_name = my_uart_name;
	baudrate  = my_baudrate;
}

Serial_Port::Serial_Port()
{
	initialize_defaults();
}

Serial_Port::~Serial_Port()
{
	pthread_mutex_destroy(&lock);
}

void Serial_Port::initialize_defaults()
{
	fd     = -1;
	is_open = false;

	uart_name = (char*)"/dev/ttyUSB0";
	baudrate  = 115200;

	int result = pthread_mutex_init(&lock, NULL);
	if ( result != 0 )
	{
		fprintf(stderr, "\n mutex init failed\n");
		throw 1;
	}
}

int Serial_Port::read_message(mavlink_message_t &message)
{
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;

	int result = _read_port(cp);
	if (result > 0)
	{
		msgReceived = mavlink_parse_char(MAVLINK_COMM_0, cp, &message, &status);
			if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
				{
					fprintf(stderr, "ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
					unsigned char v=cp;
					fprintf(stderr,"%02x ", v);
				}
				lastStatus = status;
	}
	else
	{
		fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
	}
			if(msgReceived && debug)
			{
				printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
				fprintf(stderr,"Received serial data: ");
				unsigned int i;
				uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
				unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
				if (messageLength > MAVLINK_MAX_PACKET_LEN)
				{
					fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
				}
				else
				{
					for (i=0; i<messageLength; i++)
					{
						unsigned char v=buffer[i];
						fprintf(stderr,"%02x ", v);
					}
					fprintf(stderr,"\n");
				}
			}
	return msgReceived;
}

int Serial_Port::write_message(const mavlink_message_t &message)
{
	char buf[300];
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
	int bytesWritten = _write_port(buf,len);
	return bytesWritten;
}

void Serial_Port::start()
{
	fprintf(stderr, "UART: OPEN PORT\n");
	fd = _open_port(uart_name);
	if (fd == -1)
	{
		fprintf(stderr, "failure, could not open port.\n");
		throw EXIT_FAILURE;
	}
	bool success = _setup_port(baudrate, 8, 1, false, false);
	if (!success)
	{
		fprintf(stderr, "failure, could not configure port.\n");
		throw EXIT_FAILURE;
	}
	if (fd <= 0)
	{
		fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		throw EXIT_FAILURE;
	}
	fprintf(stderr, "Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	lastStatus.packet_rx_drop_count = 0;
	is_open = true;
	fprintf(stderr, "\n");
	return;
}

void Serial_Port::stop()
{
	fprintf(stderr, "CLOSE PORT\n");
	int result = close(fd);
	if ( result )
	{
		fprintf(stderr,"WARNING: Error on port close (%i)\n", result );
	}
	is_open = false;
	fprintf(stderr, "\n");
}

int Serial_Port::_open_port(const char* port)
{
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		return(-1);
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
	return fd;
}

bool Serial_Port::_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10;
	switch (baud)
	{
		case 9600:
			if (cfsetispeed(&config, B9600) < 0 || cfsetospeed(&config, B9600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;
			break;
	}
	if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}

int Serial_Port::_read_port(uint8_t &cp)
{
	pthread_mutex_lock(&lock);
	int result = read(fd, &cp, 1);
	pthread_mutex_unlock(&lock);
	return result;
}

int Serial_Port::_write_port(char *buf, unsigned len)
{
	pthread_mutex_lock(&lock);
	const int bytesWritten = static_cast<int>(write(fd, buf, len));
	tcdrain(fd);
	pthread_mutex_unlock(&lock);
	return bytesWritten;
}


