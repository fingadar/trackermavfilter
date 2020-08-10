# trackermavfilter
 small Mavlink filter, inputs Mavlink(2), outputs few messages for i.e. trackers as mavlink1
 
 Can read Mavlink1/2 from:
  udp
  stdio
  
 Outputs only Mavlink messages MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_MSG_ID_ATTITUDE to:
  uart

Lots of trackers (i.e. U360GTS) do have problems with Mavlink2, and high unfiltered Mavlink message traffic on their UART. 

This litte utility takes a Mavlink2 stream from UDP or STDIN (for OpenHD i.e.) and outputs only the tracker relevant Mavlink messages as Mavlink1 to an UART.

Usage: trackermavlinkfilter -d <devicename> -b <baudrate> -u <udp_ip> -p <udp_port> -n (-n means use of STDIN instead of UDP)
 
For Open.HD: cat /var/run/openhd/telemetryfifo6 |  /home/pi/trackermavfilter/trackermavlinkfilter -d /dev/ttyUSB0 -b 115200 -n

