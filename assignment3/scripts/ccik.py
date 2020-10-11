#!/usr/bin/env python

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
from threading import Thread, Lock
import time

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(object):
    def __init__(self):
	#Load robot from parameter server
        self.robot = URDF.from_parameter_server()

	#Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

	#This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

	#This is a mutex
        self.mutex = Lock()

	#Subscribers and publishers for for cartesian control
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
        self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
        self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
	joint_transforms, b_T_ee = self.forward_kinematics(self.q_current)
	r_desire = numpy.zeros(4)
	t_desire = numpy.zeros(3)
	r_desire[0] = command.x_target.rotation.x
	r_desire[1] = command.x_target.rotation.y
	r_desire[2] = command.x_target.rotation.z
	r_desire[3] = command.x_target.rotation.w
	t_desire[0] = command.x_target.translation.x
	t_desire[1] = command.x_target.translation.y
	t_desire[2] = command.x_target.translation.z
	r = tf.transformations.quaternion_matrix(r_desire)
	t = tf.transformations.translation_matrix(t_desire)

	T_des = numpy.dot(t,r)
 
        r0_des = command.q0_target

        J = self.get_jacobian(b_T_ee, joint_transforms)
	jjia= numpy.linalg.pinv(J)
	jjias= numpy.linalg.pinv(J, 0.01)
	

	#x = numpy.ones(6)
	Tm = numpy.dot(tf.transformations.inverse_matrix(b_T_ee), T_des)
	trans = tf.transformations.translation_from_matrix(Tm)
	#x[0:3] = trans
	angle, axis = self.rotation_from_matrix(Tm)
	rot = numpy.dot(angle,axis)
	#x[3:6] = rot

	
	
	#ul = numpy.ones(6)
	#xu = 1/(10*math.sqrt(x[0]**2+x[1]**2+x[2]**2))*numpy.ones(3)
	#xl = numpy.ones(3)
	#ul[0:3] = xu
	#ul[3:6] = xl
	vee = numpy.transpose(numpy.hstack((trans,rot)))
	

	qd = numpy.dot(jjias, vee)

	
	if command.secondary_objective == True:
		qsec = numpy.zeros(self.num_joints)
		qsec[0] = 3*(r0_des - self.q_current[0])
		I = numpy.identity(self.num_joints)
		N = I - numpy.dot(jjias, J)
		nu = numpy.dot(N, qsec)
	        qd = qd + nu 
	qd1 = qd
	#qd1 = qd/max(qd)

	yy= JointState()
	yy.name = self.joint_names
	yy.velocity =qd1 

	self.velocity_pub.publish(yy)
	
      
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
	for i in range (0, self.num_joints):
		#Vi = numpy.zeros((3,2))
 		j_T_ee = numpy.dot(tf.transformations.inverse_matrix(joint_transforms[i]), b_T_ee)
		j_t_ee =  tf.transformations.translation_from_matrix(j_T_ee)

		inversej_T_ee = tf.transformations.inverse_matrix(j_T_ee)
		inversej_r_ee = inversej_T_ee[:3,:3]
		

		#R1 = j_T_ee[:3, :3]
		#R = numpy.transpose(R1)
		
                s = numpy.array([[0, -j_t_ee[2], j_t_ee[1]],
				 [j_t_ee[2], 0, -j_t_ee[0]],
				 [-j_t_ee[1], j_t_ee[0], 0]])
		ys = numpy.dot(-inversej_r_ee, s)
		
		#Vi[:3,0] = ys[:, 2]
		#Vi[:3,1] = inversej_r_ee[:, 2]
		v = numpy.vstack((ys, inversej_r_ee))

		vv = numpy.dot(v, self.joint_axes[i])
		J[:,i] = vv


        #--------------------------------------------------------------------------
        return J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE

	r_desire = numpy.zeros(4)
	t_desire = numpy.zeros(3)
	r_desire[0] = command.rotation.x
	r_desire[1] = command.rotation.y
	r_desire[2] = command.rotation.z
	r_desire[3] = command.rotation.w
	t_desire[0] = command.translation.x
	t_desire[1] = command.translation.y
	t_desire[2] = command.translation.z
	r = tf.transformations.quaternion_matrix(r_desire)
	t = tf.transformations.translation_matrix(t_desire)

	T_des = numpy.dot(t,r)

	times =0
	while times<3:
		qc = numpy.ones(self.num_joints)

		for i in range(0, self.num_joints):
			qc[i] = 2*math.pi*random.random()
		t1 = time.time()

		joint_transforms,b_T_ee = self.forward_kinematics(qc)
		J = self.get_jacobian(b_T_ee, joint_transforms)
		jjia= numpy.linalg.pinv(J)
		jjias= numpy.linalg.pinv(J, 0.01)
		Tm = numpy.dot(tf.transformations.inverse_matrix(b_T_ee), T_des)
		trans = tf.transformations.translation_from_matrix(Tm)
		angle, axis = self.rotation_from_matrix(Tm)
		rot = numpy.dot(angle,axis)

		
		x = numpy.hstack((trans, rot))
		dq = numpy.dot(jjias, x)

		qc = qc +dq
		while numpy.linalg.norm(x[:3])>0.001 and numpy.linalg.norm(x[3:6])>0.001:
			joint_transforms,b_T_ee = self.forward_kinematics(qc)

			#xc = numpy.ones(6)
			#xc[:3] = tf.transformations.translation_from_matrix(b_T_ee)
			#angle, axis = self.rotation_from_matrix(b_T_ee)
			#xc[3:6] = numpy.dot(angle, axis)

			#xd = numpy.ones(6)
			#xd[:3] = tf.transformations.translation_from_matrix(T_des)
			#angle, axis = self.rotation_from_matrix(T_des)
			#xd[3:6] = numpy.dot(angle, axis)

		
			J = self.get_jacobian(b_T_ee, joint_transforms)
			jjia= numpy.linalg.pinv(J)
			jjias= numpy.linalg.pinv(J, 0.01)
			Tm = numpy.dot(tf.transformations.inverse_matrix(b_T_ee), T_des)
			angle, axis = self.rotation_from_matrix(Tm)
			rot = numpy.dot(angle,axis)
			trans = tf.transformations.translation_from_matrix(Tm)
			x = numpy.hstack((trans, rot))
			
			dq = numpy.dot(jjias, x)

			qc = qc +dq
			if time.time() - t1>10:
				break
			
		if numpy.linalg.norm(dq)<0.01:
			break
			#xc = numpy.ones(6)
			#xc[:3] = tf.transformations.translation_from_matrix(b_T_ee)
			#angle, axis = self.rotation_from_matrix(b_T_ee)
			#xc[3:6] = numpy.dot(angle, axis)
		times = times+1
	qd = qc

	ll= JointState()
	ll.name = self.joint_names
	ll.position =qd

	self.joint_command_pub.publish(ll)
	

        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each joint of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.transformations.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
