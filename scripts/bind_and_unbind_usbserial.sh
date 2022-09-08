echo "Unbind USB0"
echo -n "2-2:1.0" > /sys/bus/usb/drivers/ftdi_sio/unbind 
sleep 5
echo "Bind USB0"
echo -n "2-2:1.0" > /sys/bus/usb/drivers/ftdi_sio/bind
sleep 5
