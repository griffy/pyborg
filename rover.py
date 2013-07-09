#!/usr/bin/env python3

import quick2wire.i2c as i2c
import time
import sys
import getch

ADDRESS       = 0x04
TWCR_REGISTER = 0xBC
TWSR_REGISTER = 0xB9
TWDR_REGISTER = 0xBB
TWBR_REGISTER = 0xB8

TWDR_REGISTER = 0x00

PROTO_FUNC_FORWARD        = 0x01
PROTO_FUNC_BACKWARD       = 0x02
PROTO_FUNC_LEFT_FORWARD   = 0x03
PROTO_FUNC_LEFT_BACKWARD  = 0x04
PROTO_FUNC_RIGHT_FORWARD  = 0x05
PROTO_FUNC_RIGHT_BACKWARD = 0x06
PROTO_FUNC_LEFT_ENCODER   = 0x07
PROTO_FUNC_RIGHT_ENCODER  = 0x08
PROTO_FUNC_ENCODERS       = 0x09

PROTO_FLAG_SLOW   = 0x00
PROTO_FLAG_MEDIUM = 0x01
PROTO_FLAG_FAST   = 0x02
PROTO_FLAG_FULL   = 0x03
PROTO_FLAG_STOP   = 0x04

PROTO_FLAG_RESET  = 0x00
PROTO_FLAG_READ   = 0x01

TIMEOUT = 0.25

def create_request(func, flags):
	req = func
	req <<= 4
	req |= flags
	return req

class FIRFilter(object):
	def __init__(self, order, taps=None):
		self.order = order # number of taps - 1
		self.samples = [0.0 for i in range(self.order + 1)]
		self.taps = taps # the coefficients
		self.gain = 1
		self._current_sample_index = 0

	def filter(value):
		# add the value to the sample list
		self.samples[self._current_sample_index] = value

		filtered_value = 0.0
		for i in range(self.order + 1):
			sample_index = (i + self._current_sample_index) % (self.order + 1)
			filtered_value += self.taps[i] * self.samples[sample_index]

		self._current_sample_index = (self._current_sample_index + 1) % (self.order + 1)

		return filtered_value / gain

class Robot(object):
	def __init__(self):
		self.bus = i2c.I2CMaster()
		self.left_ticks = 0
		self.right_ticks = 0
		self.update_frequency = 20 # times a second

	def __del__(self):
		self.bus.close()

	def _send(self, request):
		self.bus.transaction(
			i2c.writing_bytes(ADDRESS, request)
		)

	def update_readings(self):
		self.left_ticks, self.right_ticks = self.get_ticks()

	def get_ticks(self):
		request = create_request(PROTO_FUNC_ENCODERS, PROTO_FLAG_READ)
		results = self.bus.transaction(
			i2c.writing_bytes(ADDRESS, request),
			i2c.reading(ADDRESS, 8)
		)

		left_tick_bytes = [results[0][i] for i in range(4)]
		right_tick_bytes = [results[0][i] for i in range(4, 8)]

		left_ticks = int.from_bytes(left_tick_bytes, byteorder='little', signed=True)
		right_ticks = int.from_bytes(right_tick_bytes, byteorder='little', signed=True)

		return left_ticks, right_ticks

	def reset_ticks(self):
		self._send(create_request(PROTO_FUNC_ENCODERS, PROTO_FLAG_RESET))
		
	def left_forward(self, speed=PROTO_FLAG_FAST):
		self._send(create_request(PROTO_FUNC_LEFT_FORWARD, speed))

	def right_forward(self, speed=PROTO_FLAG_FAST):
		self._send(create_request(PROTO_FUNC_RIGHT_FORWARD, speed))

	def left_backward(self, speed=PROTO_FLAG_FAST):
		self._send(create_request(PROTO_FUNC_LEFT_BACKWARD, speed))

	def right_backward(self, speed=PROTO_FLAG_FAST):
		self._send(create_request(PROTO_FUNC_RIGHT_BACKWARD, speed))

	def move_forward(self, speed=PROTO_FLAG_FAST):
		self._send(create_request(PROTO_FUNC_FORWARD, speed))

	def move_backward(self, speed=PROTO_FLAG_FAST):
		self._send(create_request(PROTO_FUNC_BACKWARD, speed))

	def stop(self):
		self._send(create_request(PROTO_FUNC_FORWARD, PROTO_FLAG_STOP))

	def run(self, algorithm_class, *args, **kwargs):
		algorithm = algorithm_class(self, *args, **kwargs)
		while True:
			bad_update = False
			try:
				self.update_readings()
			except:
				bad_update = True
			algorithm.iterate(bad_update)
			time.sleep(1.0 / self.update_frequency)

class RobotAlgorithm(object):
	def iterate(self, bad_update):
		raise Exception("Implement me!")

class StupidSquare(RobotAlgorithm):
	def __init__(self, robot, cutoff=10):
		self.robot = robot
		self.max_iterations = robot.update_frequency * cutoff
		self.current_iteration = 0
		self.turning = False
		self.turning_counter = 0
		self.straight_counter = 0
		self.robot.reset_ticks()

	def iterate(self, bad_update):
		if self.current_iteration == self.max_iterations:
			self.robot.stop()
			sys.exit(0)

		print("%d, %d" % (self.robot.left_ticks, self.robot.right_ticks))

		if self.turning:
			self.turning_counter += 1
			if self.turning_counter % (2 * robot.update_frequency) == 0:
				self.turning = False
				self.turning_counter = 0
			self.robot.left_forward(PROTO_FLAG_FULL)
			self.robot.right_backward(PROTO_FLAG_FULL)
		else:
			self.straight_counter += 1
			if self.straight_counter % (2 * robot.update_frequency) == 0:
				self.turning = True
				self.straight_counter = 0
			self.robot.move_forward(PROTO_FLAG_FULL)

		self.current_iteration += 1

class TickSampleCollector(RobotAlgorithm):
	def __init__(self, robot, num_sample_sets=10):
		self.robot = robot
		self.num_sample_sets = num_sample_sets
		self.current_sample_set = 0
		self.current_iteration = 0
		self.left_sample_sets = [[] for i in range(num_sample_sets)]
		self.right_sample_sets = [[] for i in range(num_sample_sets)]
		self.robot.reset_ticks()

	def iterate(self, bad_update):
		if self.current_sample_set == self.num_sample_sets:
			self.robot.stop()
			sys.exit(0)

		self.left_sample_sets[self.current_sample_set].append(self.robot.left_ticks)
		self.right_sample_sets[self.current_sample_set].append(self.robot.right_ticks)

		if self.current_iteration == self.robot.update_frequency:
			self.current_sample_set += 1

class RemoteControl(RobotAlgorithm):
	def __init__(self, robot):
		self.robot = robot
		self.stopped = True

	def iterate(self, bad_update):
		direction = getch.getch()
		if direction == '' or direction == 'q':
			if not self.stopped:
				self.robot.stop()
				self.stopped = True
			if direction == 'q':
				sys.exit(0)
		else:
			self.stopped = False

			if direction == 'w':
				self.robot.move_forward(PROTO_FLAG_FULL)
			elif direction == 'a':
				self.robot.right_forward(PROTO_FLAG_FULL)
				self.robot.left_backward(PROTO_FLAG_FULL)
			elif direction == 's':
				self.robot.move_backward(PROTO_FLAG_FULL)
			elif direction == 'd':
				self.robot.left_forward(PROTO_FLAG_FULL)
				self.robot.right_backward(PROTO_FLAG_FULL)

if __name__ == "__main__":
	robot = Robot()
	robot.run(RemoteControl)
