#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018

import math
import numpy
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.01

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)

    # This function gets called every time the robot publishes its control
    # input and sensory output. You must make use of what you know about
    # extended Kalman filters to come up with an estimate of the current
    # state of the robot and covariance matrix.
    # The SensorData message contains fields 'vel_trans' and 'vel_ang' for
    # the commanded translational and rotational velocity respectively.
    # Furthermore, it contains a list 'readings' of the landmarks the
    # robot can currently observe
    def estimate(self, sens):

        #### ----- YOUR CODE GOES HERE ----- ####
	t = 0.01
	x_pred = self.x[0] + t * sens.vel_trans * math.cos(self.x[2])
	y_pred = self.x[1] + t * sens.vel_trans * math.sin(self.x[2])
	theta_pred = self.x[2] +t * sens.vel_ang
	F = [[1,0,((-t)*sens.vel_trans*math.sin(self.x[2]))], [0, 1, (t*sens.vel_trans*math.cos(self.x[2]))], [0,0,1]]
	P_pred = numpy.dot(numpy.dot(F, self.P), numpy.transpose(F)) +self.V

	x_hat=[]
	x_hat.append(x_pred)
	x_hat.append(y_pred)
	x_hat.append(theta_pred)
	xl = []
	yl = []
	y = []
	sens_read = sens.readings
	for i in range(len(sens_read)):
		xy = [x_pred[0],y_pred[0]]
		yy = [sens_read[i].landmark.x,sens_read[i].landmark.y]
		juli = numpy.linalg.norm(numpy.array(xy)-numpy.array(yy))
		juli2 = sens_read[i].range
		if juli >0.1:
			xl.append(sens_read[i].landmark.x)
			yl.append(sens_read[i].landmark.y)
			y.append(sens_read[i].range)
			y.append(sens_read[i].bearing)

	if len(xl)>0:
		H = numpy.ones((len(xl)*2,3))
		W = numpy.zeros((2*len(xl),2*len(xl)))
		y_hat = numpy.ones((2*len(xl),1))
        	for i in range(0,len(xl)):

			H[2*i][0]=(x_pred-xl[i])/math.sqrt((x_pred - xl[i])*(x_pred - xl[i])+(y_pred - yl[i])*(y_pred - yl[i]))
			H[2*i][1]=(y_pred-yl[i])/math.sqrt((y_pred - yl[i])*(y_pred - yl[i])+(x_pred - xl[i])*(x_pred - xl[i]))
			H[2*i][2]=0

			H[2*i+1][0]=(yl[i] - y_pred)/((xl[i]-x_pred)*(xl[i]-x_pred)+(yl[i]-y_pred)*(yl[i]-y_pred))


			H[2*i+1][1]=(x_pred - xl[i])/((xl[i]-x_pred)*(xl[i]-x_pred)+(yl[i]-y_pred)*(yl[i]-y_pred))

			H[2*i+1][2] =-1
			W[2*i][2*i]=0.1
			W[2*i+1][2*i+1]=0.05
			y_hat[2*i] = math.sqrt((x_pred-xl[i])*(x_pred - xl[i])+(y_pred - yl[i])*(y_pred - yl[i]))
			y_hat[2*i+1] =math.atan2(yl[i]-y_pred, xl[i]-x_pred) -theta_pred

		y_hat = numpy.transpose(y_hat)
		nu = y - y_hat

       		for i in range(0,len(xl)):
			yu = nu[0][2*i+1]%(2*numpy.pi)
			nu[0][2*i+1] = yu
			
			if nu[0][2*i+1]  > numpy.pi:
				
				nu[0][2*i+1] = yu-2*numpy.pi
			if nu[0][2*i+1] < -numpy.pi:
				
				nu[0][2*i+1] = yu+2*numpy.pi

		
		
		S = numpy.dot(numpy.dot(H,P_pred),numpy.transpose(H))+ W
		R = numpy.dot(numpy.dot(P_pred, numpy.transpose(H)), numpy.linalg.pinv(S))
		self.x = x_hat + numpy.dot(R,numpy.transpose(nu))
		d = numpy.dot(R,H)
		self.P = P_pred -numpy.dot(numpy.dot(R,H), P_pred)
	
    	else:
		
		
		self.x = x_hat
		self.P = P_pred

        #### ----- YOUR CODE GOES HERE ----- ####

    def sensor_callback(self,sens):

        # Publish state estimate
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
