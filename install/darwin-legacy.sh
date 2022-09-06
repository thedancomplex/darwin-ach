#/bin/sh -e
setserial /dev/ttyUSB0 low_latency
#        sudo setserial /dev/ttyUSB0 low_latency
sync
#        sudo sync
exit 0
