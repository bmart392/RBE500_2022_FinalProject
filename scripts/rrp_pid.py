#!/usr/bin/env python3

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import JointState
# from rrpIK.srv import rrpIK

class positionController():
	def __init__(self):
		rospy.init_node("position_controller")
		rospy.loginfo("Press Ctrl + C to terminate")

		# IK_SERVICE_NAME = "inverse_kinematics"

		# rospy.wait_for_service(IK_SERVICE_NAME)

		# self.ik_service = rospy.ServiceProxy(IK_SERVICE_NAME, rrpIK)

		self.numJoints = 3

		self.numPoints = 4

		self.jointEfforts = [0] * self.numJoints 
		self.joint_pubs = [0] * self.numJoints
		self.rate = rospy.Rate(100)

		self.joint_controllers = [0] * self.numJoints

		self.jointErrorWithinBounds = [False] * self.numJoints

		#									 8.5, .75, 200
		self.joint_controller_parameters = [[20, .000, 200], [5.0, .000, 200.0], [80., 1.11, 25.0]]
		# self.joint_controller_parameters = [[0, .000, 00], [.0, .000, 00.0], [80., 1.11, 25.0]]
		self.joint_errorBands = [.15, .15, .05]

		for i in range(self.numJoints):
			
			# Initialize publishers for each joint
			self.jointEfforts[i] = Float64()
			self.joint_pubs[i] = rospy.Publisher("/rrp/joint"+str(i+1)+"_effort_controller/command", Float64, queue_size = 10)

			# Initialize PID controllers for each joint			
			self.joint_controllers[i] = Controller()
			self.joint_controllers[i].setPID(self.joint_controller_parameters[i])

		degToRad = pi / 180

		# Generate path
		self.possiblePoints = [[0, 0.77, 0.34], \
							[-0.345, 0.425, 0.24], \
							[-0.67, -0.245, 0.14], \
							[0.77, 0.0, 0.39]]

		self.testIKSolutions = [[90 * degToRad, 00 * degToRad, 0.1], \
								[90 * degToRad, 90 * degToRad, .2], \
								[45 * degToRad, 45 * degToRad, .3], \
								[0, 0 , .05]]
		
		self.endEffectorPath = [0] * self.numPoints
		self.destinationsReached = [False] * self.numPoints

		for i in range(self.numPoints):
			self.endEffectorPath = self.possiblePoints[i]

		# subscribe to joint messages
		self.joint_status = JointState()
		self.logging_counter = 0
		self.trajectory = list()
		self.joint_status_sub = rospy.Subscriber("/rrp/joint_states", JointState, self.joint_state_callback)

		
		self.firstPassOnDestination = True
		self.destinationIndex = 0

		self.errorBand = .01


		self.trajectoryCompleteFlag = False

		self.numTimesAtIteration = 0

		self.goalTimesAtIteration = 20

		self.jointPositions = [0] * self.numJoints


		
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

		print("Current Destination Reached: " + str(self.destinationsReached[self.destinationIndex]))
		# if it is the first time through the function and the destination has not been reached yet
		if self.firstPassOnDestination and not self.destinationsReached[self.destinationIndex]:
			
			self.runFirstPassForDestination()

		else:

			# Run PID for each joint and check if the destination has been reached
			destinationReached = self.runPIDForEachJoint()
		
			# If the destination has been reached
			if destinationReached:
				
				# If the robot has remained at the target for the goal number of iterations
				if self.numTimesAtIteration >= self.goalTimesAtIteration:

					self.runLastPassForDestination()
					
				# Else keep counting the number of iterations we have reached the goal
				else:
					self.numTimesAtIteration = self.numTimesAtIteration + 1

			# Else reset the iteration counter, 
			else:
				self.numTimesAtIteration = 0

		# Always publish the joint efforts
		for i in range(self.numJoints):
			self.joint_pubs[i].publish(self.jointEfforts[i])


	def runPIDForEachJoint(self):

		# Calculate new joint efforts and  error bounding flags
		for i in range(self.numJoints):

			controller = self.joint_controllers[i]
						
			# Calculate new joint effort from controller
			self.jointEfforts[i].data = controller.update(self.jointPositions[i])
			
			# Check if the joint position has been reached
			self.jointErrorWithinBounds[i] = abs(controller.getPreviousError()) <= self.joint_errorBands[i]

			if i == 6:
				print("Joint "+str(i+1))
				print("Error: "+str(controller.getPreviousError()))
				print("Force Applied: "+str(self.jointEfforts[i].data))

		# Check if all joint destinations have been reached
		destinationReached = True

		for status in self.jointErrorWithinBounds:
			destinationReached = destinationReached and status

		return destinationReached


	def runFirstPassForDestination(self):

		# Calculate Inverse Kinematics of destination
		self.jointDestinations = self.IKserverDummy(self.endEffectorPath[self.destinationIndex], self.destinationIndex)
		# self.jointDestinations = self.ik_services.call(rrpIK(self.endEffectorPath[self.destinationIndex]))

		for i in range(self.numJoints):
			self.joint_controllers[i].setPoint(self.jointDestinations[i])

		self.firstPassOnDestination = False

		self.numTimesAtIteration = 0


	def runLastPassForDestination(self):
		# set the complete flag for the point as true
		self.destinationsReached[self.destinationIndex] = True

		# reset first pass flag
		self.firstPassOnDestination = True

		# increment the index
		self.destinationIndex = self.destinationIndex + 1

		if self.destinationIndex >= self.numPoints:
			self.trajectoryCompleteFlag = True


	def joint_state_callback(self, msg):
		# Get position for each joint
		for i in range(self.numJoints):
			self.jointPositions[i] = float(msg.position[i])

		# logging once every 100 times
		self.logging_counter += 1
		if self.logging_counter == 100000000:
			self.logging_counter = 0
			# save trajectory
			self.trajectory.append(self.jointPositions)
			rospy.loginfo("Joint 1 =" + str(self.jointPositions[0]*180/pi) + \
				"; Joint 2 = " + str(self.jointPositions[1]*180/pi) + \
				"; Joint 3 =" + str(self.jointPositions[2]))

	def IKserverDummy(self, coordinate, pointIndex):
		return self.testIKSolutions[pointIndex]

class Controller:
	def __init__(self, P=0.0, I=0.0, D=0.0, set_point=0, error_band=0):
		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.set_point = set_point # reference (desired value)
		self.numPreviousErrorSaved = 25
		self.previous_error = [0] * self.numPreviousErrorSaved # list for storing previous data points
		self.time_between_positions = 0.1 # seconds
		self.errorBand = error_band
	
	def update(self, current_value):
			
		# print("Current location is: " + str(current_value))
		# print("Set point is: " + str(self.set_point))

		error = self.set_point - current_value

		# print("Error: " + str(error))
	
		# calculate P_term, I_term, and D_term	
		P_term = error * self.Kp
		
		D_term = ((error - self.previous_error[0]) / self.time_between_positions) * self.Kd

		# print("Length of error list: " + str(len(self.previous_error)))
		sumError = 0
		for previousErrorInList in self.previous_error:
			sumError = sumError + previousErrorInList
		
		# print("Summed Error: " + str(sumError))
		
		I_term = self.Ki * sumError

		self.previous_error.insert(0, error)

		removedItem = self.previous_error.pop()

		# print("P Term: " + str(P_term))
		# print("I Term: " + str(I_term))
		# print("D Term: " + str(D_term))

		# print("Force Applied: " + str(P_term + D_term + I_term))
		returnValue = P_term + D_term + I_term

		
		return returnValue
	
	def setPoint(self, set_point):
		self.set_point = set_point
		self.previous_error = [0] * self.numPreviousErrorSaved
	
	def setPID(self, parameters):
		self.Kp = parameters[0]		
		self.Ki = parameters[1]
		self.Kd = parameters[2]

	def getPreviousError(self):
		return self.previous_error[0]



if __name__ == "__main__":
	whatever = positionController()