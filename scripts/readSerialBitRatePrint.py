#!/usr/bin/python3
import serial
import time
ser = serial.Serial('/dev/ttyUSB0', 1000000, timeout=0)
	#x = ser.read()
	#s = ser.read(10)

ser.close()
ser.open()

bytes_i = 0

dt = 0.0

tick = time.time()
tock = time.time()

while(True):
  L = ser.inWaiting()
  line = ser.read(L)
  if (L > 0):   
    bytes_i = bytes_i + L 
    print(line, end='')
    print(len(line))
  tock = time.time()
  dt = tock - tick
'''
  if (dt > 1.0):
    bytes_per_second = bytes_i / dt
    bits_per_second = bytes_per_second * 10
    bits_per_second_percentage = bits_per_second / 1000000.0 * 100.0
    print("Bits Per Second: ",end='')
    print(bits_per_second, end='')
    print(" Percentage Used of 1M: ", end='')
    print(bits_per_second_percentage, end='')
    print("%")
    bytes_i = 0
    tick = tock
'''
