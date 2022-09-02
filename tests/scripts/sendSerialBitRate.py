#!/usr/bin/python3
import serial
import time
ser = serial.Serial('/dev/ttyUSB0', 1000000, timeout=1)
	#x = ser.read()
	#s = ser.read(10)

ser.close()
ser.open()

bytes_i = 0

dt = 0.0

tick = time.time()
tock = time.time()


#\xff\xff\xc8\x04\x02\x03\x01-\xff\xff\xc8\x03\x00\xc8l\x00

#buff_tx = bytearray.fromhex("ff ff c8 04 02 03 01")
buff_tx = bytearray.fromhex("ff ff c8 03 02 03 2f")
buff_rx = bytearray.fromhex("ff ff c8 03 00 c8 00")


while(True):
  ser.write(buff_tx)
  time.sleep(1/1600.0)
  tock = time.time()
  dt = tock - tick
  print(dt, end='')
  print(" f = ", end='')
  print(1/dt)
  tick = tock
