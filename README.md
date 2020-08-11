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


Example installation in Open.HD:

1. Copy the binary to /usr/local/bin/

2. Add a parameter to openhd-settings-?.txt:
TELEMETRY_OUTPUT_SERIALPORT_GROUND_COMMAND=/usr/local/bin/trackermavlinkfilter

3. Change /usr/local/share/wifibroadcast-scripts/osd_rx_functions.sh:

        if [ "${ENABLE_SERIAL_TELEMETRY_OUTPUT}" == "Y" ]; then
            if [ "${TELEMETRY_OUTPUT_SERIALPORT_GROUND_COMMAND}" != "" ]; then
                echo "Sending telemetry stream to ${TELEMETRY_OUTPUT_SERIALPORT_GROUND_COMMAND} -d ${TELEMETRY_OUTPUT_SERIALPORT_GROUND} -b ${TELEMETRY_OUTPUT_SERIALPORT_GROUND_BAUDRATE} -n"

                nice cat /var/run/openhd/telemetryfifo6 | ${TELEMETRY_OUTPUT_SERIALPORT_GROUND_COMMAND} -d ${TELEMETRY_OUTPUT_SERIALPORT_GROUND} -b ${TELEMETRY_OUTPUT_SERIALPORT_GROUND_BAUDRATE} -n &
            else
                echo "Sending telemetry stream to ${TELEMETRY_OUTPUT_SERIALPORT_GROUND}"
        
                nice stty -F ${TELEMETRY_OUTPUT_SERIALPORT_GROUND} ${TELEMETRY_OUTPUT_SERIALPORT_GROUND_STTY_OPTIONS} ${TELEMETRY_OUTPUT_SERIALPORT_GROUND_BAUDRATE}
        
                nice cat /var/run/openhd/telemetryfifo6 > ${TELEMETRY_OUTPUT_SERIALPORT_GROUND} &
            fi
        fi
