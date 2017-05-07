#!/usr/bin/env python

from time import sleep
import serial

#ser = serial.Serial('/dev/tty.usbmodem1d11', 9600)
ser_arduino = serial.Serial('/dev/ttyUSB0', 115200)

arduino_string = ""
raw_data_array = []
raw_nb = 0
raw_ax = 0.0
raw_ay = 0.0
raw_az = 0.0
raw_gx = 0.0
raw_gy = 0.0
raw_gz = 0.0

#ser_arduino = serial.Serial('com11', 115200) #Creating our serial object named arduinoData

"""	------------------------------
	SERIAL COMMUNICATION FUNCTIONS
	------------------------------ """
def read_robot_data():
	global arduino_string
	global raw_data_array
	arduino_string = ser_arduino.readline()
	raw_data_array = arduino_string.split()
def decode_robot_data():
	global raw_nb
	global raw_ax
	global raw_ay
	global raw_az
	global raw_gx
	global raw_gy
	global raw_gz
	raw_nb = int(raw_data_array[0])
	raw_ax = float(raw_data_array[1])
	raw_ay = float(raw_data_array[2])
	raw_az = float(raw_data_array[3])
	raw_gx = float(raw_data_array[4])
	raw_gy = float(raw_data_array[5])
	raw_gz = float(raw_data_array[6])
def print_robot_data():
	print "----------- New IMU data nb " + str(raw_nb) + " acquired -----------"
	print "Raw data: Acceleration (x,y,z): (" + str(raw_ax) + "," + \
							str(raw_ay) + "," + str(raw_az) + ")"
	print "Raw data: Gyroscope    (x,y,z): (" + str(raw_gx) + "," + \
							str(raw_gy) + "," + str(raw_gz) + ")"



"""	-----------------------------------
	COMP3211 FINAL PROJECT INSTRUCTIONS
	----------------------------------- """
def comp3211_project():
	counter = 32 	# Below 32 everything in ASCII is gibberish

	#	------------------------------------------------------------------------
	#	When starting to read from serial port, it might be the middle of a line
	#	Start by removing first line
	while (ser_arduino.inWaiting()==0): #Wait here until there is data
			pass
	read_robot_data()
	try:
		print "Raw Arduino 1st Data:"
		print raw_data_array
		decode_robot_data()		#	Try to decode first array of data
	except ValueError:
		print " PROBLEM: ValueError"
		read_robot_data()		#	Read a new line to "forget" last error
	except IndexError:
		print " PROBLEM: IndexError"
		read_robot_data()

	#	---------
	#	Main loop
	while True:
		counter = counter + 1
		if counter == 256:
			counter = 32

		#	-----------------
		#	Read Arduino Data
		#while (ser_arduino.inWaiting()==0): #Wait here until there is data
			#pass
		read_robot_data()
		#print raw_data_array
		decode_robot_data()
		print_robot_data()

		#	-------------------------------------
		#	Send motor angles commands to Arduino
		#ser_arduino.write(str(chr(counter))) # Convert the decimal number to ASCII then send it to the Arduino
		print "Sending motor angles commands to Arduino"
		ser_arduino.write(str(chr(1)) + str(chr(1)) + str(chr(1)) + str(chr(1)))
		#	Read acknowledgement data sent back from arduino (ignore it)
		read_robot_data()
		print raw_data_array

		#sleep(.1) # Delay for one tenth of a second

""" Execute main code """
comp3211_project()