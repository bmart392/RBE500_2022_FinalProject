#!/usr/bin/env python3

from math import pi, sqrt, atan2, cos, sin
import numpy as np
# from controller import *

import rospy
import tf
from std_msgs.msg import Empty, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import JointState

class positionController():
	def __init__(self):
		rospy.init_node("position_controller")
		rospy.loginfo("Press Ctrl + C to terminate")

		self.numJoints = 3
		self.iterationNumJoints = self.numJoints - 1

		self.numberOfPointsInPath = 4
		self.iterationNumPoints = self.numberOfPointsInPath - 1

		self.jointEfforts = []
		self.joint_pubs = []
		self.joint_rates = []

		self.joint_controllers = []

		self.jointErrorWithinBounds = []

		self.joint_controller_parameters = [[1, 1, 1], [1, 1, 1,], [1, 1, 1]]

		for i in range(self.iterationNumJoints):
			
			# Initialize publishers for each joint
			self.jointEfforts[i] = Float64()
			self.joint_pubs[i] = rospy.publisher("/rrp/joint"+(i+1)+"_effort_controller/command", Float64, queue_size = 10)
			self.joint_rates = rospy.rate(10)

			# Initialize PID controllers for each joint			
			self.joint_controllers[i] = Controller()
			self.joint_controllers[i].setPID(self.joint_controller_parameters[i])

			self.jointErrorWithinBounds[i] = False

		# subscribe to joint messages
		self.joint_status = JointState()
		self.logging_counter = 0
		self.trajectory = list()
		self.joint_status_sub = rospy.Subscriber("/rrp/joint_states", JointState, self.joint_state_callback)

		# Generate path
		self.endEffectorPath = [[0, 0.77, 0.34], \
								[-0.345, 0.425, 0.24], \
								[-0.67, -0.245, 0.14], \
								[0.77, 0.0, 0.39]]

		self.destinationsReached = [False, False, False, False]

		self.firstPassOnDestination = True
		self.destinationIndex = 0

		self.errorBand = .01


		self.trajectoryCompleteFlag = False

		self.numTimesAtIteration = 0

		self.goalTimesAtIteration = 20


		print("Start of tragjectory movement.")
		while not rospy.is_shutdown() and not self.trajectoryCompleteFlag:
			
			# Start next iteration of control loop
			try:
				# Send commands to the robot
				self.run()

				# Sleep until ready to send next command
				self.rate.sleep()
			
			except rospy.ROSInterruptException:
				rospy.loginfo("Action terminated.")
			
			finally:
				# save trajectory into csv file
				np.savetxt("trajectory.csv", np.array(self.trajectory), fmt="%f", delimiter=",")

			
	
	def run(self):

		# if it is the first time through the function and the destination has not been reached yet
		if self.firstPassOnDestination and not self.listOfDestinations[self.destinationIndex][3]:
			
			runFirstPassForDestination()

		else:

			# Run PID for each joint and check if the destination has been reached
			destinationReached = runPIDForEachJoint()
		
			# If the destination has been reached
			if destinationReached:
				
				# If the robot has remained at the target for the goal number of iterations
				if self.numTimesAtIteration >= self.goalTimesAtIteration:

					runLastPassForDestination()
					
				# Else keep counting the number of iterations we have reached the goal
				else:
					self.numTimesAtIteration = self.numTimesAtIteration + 1

			# Else reset the iteration counter, 
			else:
				self.numTimesAtIteration = 0

		# Always publish the joint efforts
		for i in range(self.iterationNumJoints)
			self.joint_pubs[i].publish(self.jointEfforts[i])


	def runPIDForEachJoint():

		# Calculate new joint efforts and  error bounding flags
		for i in range(self.iterationNumJoints):
			controller = self.joint_controllers[i]
						
			# Calculate new joint effort from controller
			self.jointEfforts[i].data = controller.update(self.jointPositions[i])
			
			# Check if the joint position has been reached
			self.jointErrorWithinBounds[i] = abs(controller.getPreviousError()) <= self.errorBand

		# Check if all joint destinations have been reached
		destinationReached = True

		for status in self.jointErrorWithinBounds:
			destinationReached = destinationReached and status

		return destinationReached


	def runFirstPassForDestination():

		# Pull destination x, y, and z out of the end effector path
		self.destination_x = self.endEffectorPath[self.destinationIndex][1]
		self.destination_y = self.endEffectorPath[self.destinationIndex][2]
		self.destination_z = self.endEffectorPath[self.destinationIndex][3]

		# Calculate Inverse Kinematics of destination
		self.jointDestinations = sampleIK(self.destination_x, self.destination_y\
			self.destination_z)

		for controller in self.joint_controllers:
			controller.setPoint(self.joint_controllers.index(controller))

		self.firstPassOnDestination = False

		self.numTimesAtIteration = 0


	def runLastPassForDestination():
		# set the complete flag for the point as true
		self.destinationsReached[self.destinationIndex] = True

		# reset first pass flag
		self.firstPassOnDestination = True

		# increment the index
		self.destinationIndex = self.destinationIndex + 1

		if self.destinationIndex >= len(self.listOfDestinations):
			self.trajectoryCompleteFlag = True

		
	def odom_callback(self, msg):
		# get pose = (x, y, theta) from odometry topic
		quarternion = [msg.pose.pose.orientation.x, \
						msg.pose.pose.orientation.y, \
						msg.pose.pose.orientation.z, \
						msg.pose.pose.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
			quarternion)
		self.pose.theta = yaw
		self.pose.x = msg.pose.pose.position.x
		self.pose.y = msg.pose.pose.position.y
		
		# logging once every 100 times
		self.logging_counter += 1
		if self.logging_counter == 15:
			self.logging_counter = 0
			# save trajectory
			self.trajectory.append([self.pose.x, self.pose.y])
			rospy.loginfo("odom: x=" + str(self.pose.x) + \
				"; y=" + str(self.pose.y) + "; theta=" + str(yaw))

	def joint_state_callback(self, msg)
		# Get position for each joint
		self.jointPositions = msg.position

		# logging once every 100 times
		self.logging_counter += 1
		if self.logging_counter == 15:
			self.logging_counter = 0
			# save trajectory
			self.trajectory.append([self.jointPositions])
			rospy.loginfo("Joint 1 =" + str(self.jointPositions[0]*180/pi) + \
				"; Joint 2 = " + str(self.jointPositions[1]*180/pi) + \
				"; Joint 3 =" + str(self.jointPositions[2]*180/pi))


class Controller:
	def __init__(self, P=0.0, I=0.0, D=0.0, set_point=0):
		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.set_point = set_point # reference (desired value)
		self.previous_error = [] # list for storing previous data points
		self.time_between_positions = 0.1 # seconds
	
	def update(self, current_value):
			
		error = self.set_point - current_value
		
		# calculate P_term, I_term, and D_term	
		P_term = error * self.Kp
		
		D_term = ((self.previous_error - error) / self.time_between_positions) * self.Kd
		
		sumError = 0
		for error in self.previous_error:
			sumError = sumError + error
		I_term = Ki * sumError

		self.previous_error.insert(error, 0)

		if len(self.previous_error) >= 10:
			removedItem = self.previous_error.pop(10)
		return P_term + D_term + I_term
	
	def setPoint(self, set_point):
		self.set_point = set_point
		self.previous_error = 0
	
	def setPID(self, P=0.0, D=0.0, I=0.0):
		self.Kp = P
		self.Kd = D
		self.Ki = I

	def getPreviousError()
		return self.previous_error[0]



if __name__ == "__main__":
	whatever = positionController()