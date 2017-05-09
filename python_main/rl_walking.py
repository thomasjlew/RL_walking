#!/usr/bin/env python

from time import sleep
import serial

#ser = serial.Serial('/dev/tty.usbmodem1d11', 9600)
ser_arduino = serial.Serial('/dev/ttyUSB0', 115200)

#	Data received from Arduino board
arduino_string = ""
raw_data_array = []
raw_nb = 0
raw_ax = 0.0
raw_ay = 0.0
raw_az = 0.0
raw_gx = 0.0
raw_gy = 0.0
raw_gz = 0.0
time_ms  = 0.0
time_sec = 0.0

#	Robot angles and axis defintion
#		   ______					--------------------------
#		  |______| 					|^az=1     MPU    <--.   |
#		   3/  \1 		y 			||       ARDUINO   gx )  |
#		   /    \		^ 			||                \  /   |
#		   \    /		| 			|.---->ay=0        --    |
#		   4\  /2 		|-----> x   --------------------------
#
INIT_ANGLE1 = 90
INIT_ANGLE2 = 80
INIT_ANGLE3 = 90
INIT_ANGLE4 = 80
MIN_ANGLE1 = 90
MIN_ANGLE2 = 80
MIN_ANGLE3 = 90
MIN_ANGLE4 = 80
MAX_ANGLE1 = 115
MAX_ANGLE2 = 140
MAX_ANGLE3 = 130
MAX_ANGLE4 = 140

#	Measured with measure_motor_speed()
SPEED_SERVOS_RAD_SEC  = 5.233 #5.233[rad/s] = 300[deg/sec] = 0.300[deg/millisec]
SPEED_SERVOS_DEG_MSEC = 0.300
EPSILON_ANGLE = 5 # in [deg]

class Servo:
	""" Implementation of the Servo Class. """
	joint_nb = 0;
	current_angle = 90.
	goal_angle = 90.
	direction_change_angle = 0; # +1 indicates positive speed -1 indicates neg.

	def __init__(self, angle, joint_nb):
		self.current_angle = angle
		self.goal_angle = angle
		self.joint_nb = joint_nb

	def print_goal_angle(self):
		print "Servo " + str(self.joint_nb) + ", goal: " + str(self.goal_angle)
	def print_current_angle(self):
		print "Servo "+str(self.joint_nb)+", current: "+ str(self.current_angle)

	def get_goal_angle(self):
		return self.goal_angle
	def get_current_angle(self):
		return self.current_angle

	def update_current_angle(self, delta_t_ms):
		if self.direction_change_angle == 0:
			pass
		elif self.direction_change_angle == 1:
			print "increasing angle by "+str(delta_t_ms * SPEED_SERVOS_DEG_MSEC)
			self.current_angle += float(delta_t_ms) * SPEED_SERVOS_DEG_MSEC
		elif self.direction_change_angle == -1:
			print "decreasing angle by "+str(-delta_t_ms *SPEED_SERVOS_DEG_MSEC)
			self.current_angle -= float(delta_t_ms) * SPEED_SERVOS_DEG_MSEC
		if  self.current_angle <  self.goal_angle + EPSILON_ANGLE and \
			self.current_angle >= self.goal_angle - EPSILON_ANGLE:
			if self.joint_nb == 4:
				print "goal position of servo reached!"
			self.direction_change_angle = 0
			self.current_angle = self.goal_angle
	def set_goal_angle(self, angle):
		self.goal_angle = angle
		self.check_angle()	#	dont ommit check_angle() or it can self-destroy
		if self.current_angle < self.goal_angle:
			self.direction_change_angle = 1;
		elif self.current_angle > self.goal_angle:
			self.direction_change_angle = -1;
	def set_current_angle(self, angle):
		self.current_angle = angle

	def check_angle(self):
		if self.joint_nb == 1:
			if self.goal_angle < MIN_ANGLE1:
				print "too small joint angle for servo 1"
				self.goal_angle = MIN_ANGLE1
			elif self.goal_angle > MAX_ANGLE1:
				print "too large joint angle for servo 1"
				self.goal_angle = MAX_ANGLE1

		elif self.joint_nb == 2:
			if self.goal_angle < MIN_ANGLE2:
				print "too small joint angle for servo 2"
				self.goal_angle = MIN_ANGLE2
			elif self.goal_angle > MAX_ANGLE2:
				print "too large joint angle for servo 2"
				self.goal_angle = MAX_ANGLE2

		elif self.joint_nb == 3:
			if self.goal_angle < MIN_ANGLE3:
				print "too small joint angle for servo 3"
				self.goal_angle = MIN_ANGLE3
			elif self.goal_angle > MAX_ANGLE3:
				print "too large joint angle for servo 3"
				self.goal_angle = MAX_ANGLE3

		elif self.joint_nb == 4:
			if self.goal_angle < MIN_ANGLE4:
				print "too small joint angle for servo 4"
				self.goal_angle = MIN_ANGLE4
			elif self.goal_angle > MAX_ANGLE4:
				print "too large joint angle for servo 4"
				self.goal_angle = MAX_ANGLE4


class Robot:
	""" Implementation of the Robot Class. 
		Contains all the state variables fixed to the main body of robot."""
	a_x = 0.
	a_y = 0.
	a_z = 0.
	v_x = 0.
	v_y = 0.
	v_z = 0.
	omega_x = 0.
	omega_y = 0.
	omega_z = 0.
	back_leg_ground_touched = 0.
	front_leg_ground_touched = 0.
	lidar = 0

	def __init__(self,
				prev_a_x = 0.,
				prev_a_y = 0.,
				prev_a_z = 0.,
				v_x = 0.,
				v_y = 0.,
				v_z = 0.,
				omega_x = 0.,
				omega_y = 0.,
				omega_z = 0.,
				back_leg_ground_touched = 0.,
				front_leg_ground_touched = 0.,
				lidar = 0):
		self.a_x = prev_a_x.
		self.a_y = prev_a_y.
		self.a_z = prev_a_z.
		self.v_x = v_x.
		self.v_y = v_y.
		self.v_z = v_z.
		self.omega_x = omega_x.
		self.omega_y = omega_y.
		self.omega_z = omega_z.
		self.back_leg_ground_touched = back_leg_ground_touched.
		self.front_leg_ground_touched = front_leg_ground_touched
		self.lidar = lidar

	def print_prev_accel(self):
		print "Previous Accel (x,y,z): (" + str(self.a_x) + "," \
		 								  + str(self.a_y) + "," \
		 								  + str(self.a_z) + ")"
	def print_current_speeds(self):
		print "Current linear speeds (v_x, y, z): (" \
			+ str(self.v_x) + "," + str(self.v_y) + "," + str(self.v_z) + ")"
		print "Current angular speeds(w_x, y, z): (" \
			+ str(self.omega_x) + "," + str(self.omega_y) + "," + str(self.omega_z) + ")"

	def get_a_x(self):
		return self.a_x
	def get_a_y(self):
		return self.a_y
	def get_a_z(self):
		return self.a_z
	def get_v_x(self):
		return self.v_x
	def get_v_y(self):
		return self.v_y
	def get_v_z(self):
		return self.v_z
	def get_omega_x(self):
		return self.omega_x
	def get_omega_y(self):
		return self.omega_y
	def get_omega_z(self):
		return self.omega_z

	#	Currently, only certain values in x is updated
	def update_current_speeds(self, delta_t_ms, ay, az, omegax):
		# Apply low-pass filter to avoid spikes
		alpha = 0.3
		self.a_y = alpha*ay + (1-alpha)*self.a_y
		self.a_z = alpha*az + (1-alpha)*self.a_z
		self.omega_x = alpha*omegax + (1-alpha)*self.omega_x

		# Very raw estimation
		self.v_y = self.v_y + float(delta_t_ms)/1000 * self.a_y
		self.v_z = self.v_z + float(delta_t_ms)/1000 * self.a_z


def update_robot_angles(s1, s2, s3, s4, delta_t_ms):
	s1.update_current_angle(delta_t_ms)
	s2.update_current_angle(delta_t_ms)
	s3.update_current_angle(delta_t_ms)
	s4.update_current_angle(delta_t_ms)


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
	global time_ms
	global time_sec
	raw_nb = int(raw_data_array[0])
	raw_ax = float(raw_data_array[1])
	raw_ay = float(raw_data_array[2])
	raw_az = float(raw_data_array[3])
	raw_gx = float(raw_data_array[4])
	raw_gy = float(raw_data_array[5])
	raw_gz = float(raw_data_array[6])
	time_ms= float(raw_data_array[7])
	time_sec = time_ms / 1000

def print_robot_data(s1, s2, s3, s4):
	print "                                                      "
	print "------------------------------------------------------"
	print "----------- New IMU data nb " + str(raw_nb) + " acquired -----------"
	print "Arduino time [sec]: " + str(int(time_sec))+":"+str(int(time_ms%1000))
	print "Raw data: Acceleration (x,y,z): (" + str(raw_ax) + "," + \
							str(raw_ay) + "," + str(raw_az) + ")"
	print "Raw data: Gyroscope    (x,y,z): (" + str(raw_gx) + "," + \
							str(raw_gy) + "," + str(raw_gz) + ")"

	print "----- States of servomotors -----"
	s1.print_current_angle()
	s2.print_current_angle()
	s3.print_current_angle()
	s4.print_current_angle()
	s1.print_goal_angle()
	s2.print_goal_angle()
	s3.print_goal_angle()
	s4.print_goal_angle()


"""	------------------------
	INITIALIZATION FUNCTIONS
	------------------------ """
def init_serial_communication():
	#	------------------------------------------------------------------------
	#	When starting to read from serial port, it might be the middle of a line
	#	Start by removing this first line
	while (ser_arduino.inWaiting() == 0): #Wait here until there is data
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

""" ----------------------------------------------------------------------------
	By finding the shortest possible period to make a full movement,
	and by knowing the angle and period, it is possible to derive the
	(maximum) speed 														"""
def measure_motor_speed(s1, s2, s3, s4):
	#print "current time: " + str(int(time_sec))
	print "current time: " + str(int(time_ms))
	if int(time_sec)%2 == 0:
		#if int(time_sec)%2 == 0:
		if int(time_ms)%400 > 200:
		#if int(time_ms)%500 > 250:
		#if int(time_ms)%600 > 300:
			print "SETTING SERVO NB 4 !!!!"
			print "MIN_ANGLE4"
			#s4.set_goal_angle(MIN_ANGLE4)
			s2.set_goal_angle(MIN_ANGLE2)
		#if int(time_sec)%2 == 1:
		if int(time_ms)%400 <= 200:
		#if int(time_ms)%600 <= 300:
			print "SETTING SERVO NB 4 !!!!"
			print "MAX_ANGLE4"
			#s4.set_goal_angle(MAX_ANGLE4)
			s2.set_goal_angle(MAX_ANGLE2)
		#MIN_ANGLE4 = 80
		#MAX_ANGLE4 = 140
""" Hence, the time taken for a full movement of MAX_ANGLE4-MIN_ANGLE4 [deg] is
	speed_servo_4 = (MAX_ANGLE4-MIN_ANGLE4)/.2 = 300 [deg/sec] = 5.233 [rad/sec] 
	We assume that all servomotors have the same speed (same model & power) """





"""	-----------------------------------
	COMP3211 FINAL PROJECT INSTRUCTIONS
	----------------------------------- """
def comp3211_project():
	counter = 1
	prev_time_ms = 0;
	this_time_ms = 0;

	s1 = Servo(INIT_ANGLE1, 1)
	s2 = Servo(INIT_ANGLE2, 2)
	s3 = Servo(INIT_ANGLE3, 3)
	s4 = Servo(INIT_ANGLE4, 4)
	robot = Robot()

	init_serial_communication()

	read_robot_data()
	decode_robot_data()
	this_time_ms = time_ms

	#	---------
	#	Main loop
	while True:
		counter = counter + 1
		if counter == 256:
			counter = 1

		#	---------------------
		#	Read Arduino IMU Data
		read_robot_data()
		#print raw_data_array #should show imu data with "RockNRoll!" at the end

		#	If an error occurs here, then the python script is too slow
		#	To fix it, slow down arduino code (increase SERIAL_IN_OUT_DELAY)
		decode_robot_data()

		#	Update time and robot joint angles
		prev_time_ms = this_time_ms
		this_time_ms = time_ms
		delta_t_ms = this_time_ms - prev_time_ms
		update_robot_angles(s1, s2, s3, s4, delta_t_ms)
		robot.update_current_speeds(delta_t_ms, raw_ay, raw_az, raw_gx)
		print_robot_data(s1, s2, s3, s4)



		#	---------------------
		"""	INTELLIGENT AGENT """
		#	------------------------------------------
		#	Available data for this step:
		#		
		angle1 = s1.get_current_angle()
		angle2 = s2.get_current_angle()
		angle3 = s3.get_current_angle()
		angle4 = s4.get_current_angle()
		#	------------------------------------------
		#	Computation of good joints angles to walk!

		#measure_motor_speed(s1, s2, s3, s4)#test to measure the speed of servos

		#	------------------------------------------
		#	------------------------------------------
		#	--------intelligent agent!----------------
		#	------------------------------------------
		#	------------------------------------------
		#	------------------------------------------


		#	-------------------------------------
		#	Send motor angles commands to Arduino
		#ser_arduino.write(str(chr(counter))) # Convert the decimal number to ASCII then send it to the Arduino
		#s1.set_goal_angle(10)
		#s2.set_goal_angle(10)
		#s3.set_goal_angle(10)
		#s4.set_goal_angle(10)
		print "Sending motor angles commands to Arduino"
		ser_arduino.write(	str(s1.get_goal_angle()) + " " + \
							str(s2.get_goal_angle()) + " " + \
							str(s3.get_goal_angle()) + " " + \
							str(s4.get_goal_angle())  )

		#	Read acknowledgement data sent back from arduino (ignore it)
		read_robot_data()
		print raw_data_array

		#sleep(.1) # Delay for one tenth of a second


""" Execute main code """
comp3211_project()