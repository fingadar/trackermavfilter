mavlink_filter: trackermavlinkfilter.cpp serial_port.cpp udp_port.cpp stdio_port.cpp trackerfilter_interface.cpp 
	g++ -g -Wall -I mavlink/include/mavlink/v2.0 trackermavlinkfilter.cpp serial_port.cpp udp_port.cpp stdio_port.cpp trackerfilter_interface.cpp -o trackermavlinkfilter -lpthread

clean:
	 rm -rf *o trackermavlinkfilter
