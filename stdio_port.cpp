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
 * @file stdio_port.cpp
 *
 * @brief stdio interface functions
 *
 * Functions for reading stdio
 *
 * @author Stefan Arbes <info@unmanned-technologies.de>
 *
 */

#include "stdio_port.h"

extern bool debug; 

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
STDIO_Port:: STDIO_Port()
{
	initialize_defaults();
	is_open = false;
}

STDIO_Port::~STDIO_Port()
{
	pthread_mutex_destroy(&lock);
}

void STDIO_Port::initialize_defaults()
{
	int result = pthread_mutex_init(&lock, NULL);
	if ( result != 0 )
	{
		fprintf(stderr, "\n mutex init failed\n");
		throw 1;
	}
	buff_ptr = buff_len;
}

int STDIO_Port::read_message(mavlink_message_t &message)
{
	uint8_t  	     cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;

	if ( _read_port (cp))
	{

		msgReceived = mavlink_parse_char(MAVLINK_COMM_0, cp, &message, &status);
				if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
					{
						fprintf(stderr, "ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
						unsigned char v = cp;
						fprintf(stderr, "%02x ", v);
					}
					lastStatus = status;
				if(msgReceived && debug)
				{
					fprintf(stderr, "Received message from STDIN with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
					fprintf(stderr, "Received STDIN data: ");
					unsigned int i;
					uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
					unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
					if (messageLength > MAVLINK_MAX_PACKET_LEN)
					{
						fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
					}
					else
					{
						for (i = 0; i < messageLength; i++)
						{
							unsigned char v=buffer[i];
							fprintf(stderr,"%02x ", v);
						}
						fprintf(stderr,"\n");
					}
				}
	}
	return msgReceived;	
}

int STDIO_Port::write_message(const mavlink_message_t &message)
{
	char buf[300];
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
	_write_port (buf, len);
	return len;
}

void STDIO_Port::start()
{
	lastStatus.packet_rx_drop_count = 0;
	is_open = true;
	fprintf(stderr, "Start reading from stdin\n");	
	return;
}

void STDIO_Port::stop()
{
	is_open = false;
}

int STDIO_Port::_read_port(uint8_t &cp)
{
	pthread_mutex_lock(&lock);
	int result = -1;
	if( buff_ptr < buff_len)
	{
		cp = buff[buff_ptr];
		buff_ptr++;
		result = 1;
	} 
	else
	{
		buff_len = BUFF_LEN;
		std::cin.read (buff, buff_len);
		result = std::cin.gcount();
		if (result > 0) 
		{
			buff_len = result;
			buff_ptr = 0;
			cp = buff[buff_ptr];
			buff_ptr++;
		}
	}
	pthread_mutex_unlock(&lock);
	return result;
}

int STDIO_Port::_write_port(char *buf, unsigned len)
{
	pthread_mutex_lock(&lock);
	std::cout.write (buf, len);
	pthread_mutex_unlock(&lock);
	return len;
}


