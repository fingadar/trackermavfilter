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
 * @file udp_port.cpp
 *
 * @brief UDP interface functions
 *
 * Functions for opening, closing, reading and writing via UDP ports
 *
 * @author Stefan Arbes <info@unmanned-technologies.de>
 *
 */

#include "udp_port.h"

extern bool debug; 

UDP_Port::UDP_Port(const char *my_target_ip, int my_udp_port)
{
	initialize_defaults();
	target_ip = my_target_ip;
	rx_port  = my_udp_port;
	is_open = false;
}

UDP_Port::UDP_Port()
{
	initialize_defaults();
}

UDP_Port::~UDP_Port()
{
	pthread_mutex_destroy(&lock);
}

void UDP_Port::initialize_defaults()
{
	target_ip = "127.0.0.1";
	rx_port  = 14552;
	tx_port  = -1;
	is_open = false;
	sock = -1;

	int result = pthread_mutex_init(&lock, NULL);
	if ( result != 0 )
	{
		fprintf(stderr, "\n mutex init failed\n");
		throw 1;
	}
}

// ------------------------------------------------------------------------------
//   Read mavlink message from UDP
// ------------------------------------------------------------------------------
int UDP_Port::read_message(mavlink_message_t &message)
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
			fprintf(stderr, "%02x ", v);
		}
		lastStatus = status;
	} 
	else
	{
		fprintf(stderr, "ERROR: Could not read, res = %d, errno = %d : %m\n", result, errno);
	}

			if (msgReceived && debug)
			{
				fprintf(stderr,"Received message from UDP with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
				fprintf(stderr,"Received UDP data: ");
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
						fprintf(stderr, "%02x ", v);
					}
					fprintf(stderr, "\n");
				}
			}
	return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to UDP
// ------------------------------------------------------------------------------
int UDP_Port::write_message(const mavlink_message_t &message)
{
	char buf[300];

	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	int bytesWritten = _write_port(buf,len);
	if (bytesWritten < 0) 
	{
		fprintf(stderr, "ERROR: Could not write, res = %d, errno = %d : %m\n", bytesWritten, errno);
	}

	return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Open UDP Port
// ------------------------------------------------------------------------------

void UDP_Port::start()
{
	sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0)
	{
		perror("error socket failed");
		throw EXIT_FAILURE;
	}

	/* Bind the socket to rx_port - necessary to receive packets */
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(target_ip);;
	addr.sin_port = htons(rx_port);

	if (bind(sock, (struct sockaddr *) &addr, sizeof(struct sockaddr)))
	{
		perror("error bind failed");
		close(sock);
		sock = -1;
		throw EXIT_FAILURE;
	}

	fprintf(stderr, "Listening to %s:%i\n", target_ip, rx_port);
	lastStatus.packet_rx_drop_count = 0;
	is_open = true;
	fprintf(stderr, "\n");
	return;
}

void UDP_Port::stop()
{
	fprintf(stderr, "CLOSE PORT\n");
	int result = close(sock);
	sock = -1;
	if ( result )
	{
		fprintf(stderr,"WARNING: Error on port close (%i)\n", result );
	}
	is_open = false;
	fprintf(stderr, "\n");

}

int UDP_Port::_read_port(uint8_t &cp)
{
	socklen_t len;

	pthread_mutex_lock(&lock);

	int result = -1;
	if (buff_ptr < buff_len)
	{
		cp = buff[buff_ptr];
		buff_ptr++;
		result = 1;
	} else 
	{
		struct sockaddr_in addr;
		len = sizeof(struct sockaddr_in);
		result = recvfrom(sock, &buff, BUFF_LEN, 0, (struct sockaddr *)&addr, &len);
		if( result > 0)
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

int UDP_Port::_write_port(char *buf, unsigned len)
{
	pthread_mutex_lock(&lock);

	int bytesWritten = 0;
	if (tx_port > 0)
	{
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = inet_addr(target_ip);
		addr.sin_port = htons(tx_port);
		bytesWritten = sendto(sock, buf, len, 0, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
	}
	pthread_mutex_unlock(&lock);

	return bytesWritten;
}


