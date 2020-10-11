#!/usr/bin/env python

import numpy
import random
import sys
import math
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''  
  
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
	r = numpy.ones(4)
	t = numpy.ones(3)

	r[0] = ee_goal.rotation.x
	r[1] = ee_goal.rotation.y
	r[2] = ee_goal.rotation.z
	r[3] = ee_goal.rotation.w

	t[0] = ee_goal.translation.x
	t[1] = ee_goal.translation.y
	t[2] = ee_goal.translation.z

	rot = tf.transformations.quaternion_matrix(r)
	trans = tf.transformations.translation_matrix(t)
	goalT = numpy.dot(trans, rot)
	
 
	currentq = numpy.array(self.q_current)
	
	
	goalq = numpy.array(self.IK(goalT))
	print('start',currentq)
	print('end',goalq)
	while self.is_state_valid(goalq) == False:
		print('False')
		goalq = numpy.array(self.IK(goalT))
	
# forward tree
	tree = []
	tree.append(currentq)
	parent = []
	parent.append(0)
	print('tree',tree)

# backward tree
	treeb = []
	treeb.append(goalq)
	parentb = []
	parentb.append(0)
	print('tree back',treeb)
	xin = currentq
	xinb = goalq

	cs = 0
	cd = 0
	print('start growing trees')
# grow the tree
	left = 999
	while numpy.linalg.norm(xinb-xin)>0.01:
		if self.is_segment_valid(xinb, xin) == True:
			print('direct have a path')	
			a = len(tree)
			fangxiang = xinb-xin
			changdu = numpy.linalg.norm(xinb-xin)
			num = changdu/0.1
			num = numpy.ceil(num)
			step = fangxiang/num
			for i in numpy.arange(0, num):
				xin = xin + step
				tree.append(xin)
				parent.append(a)
				a = a + 1				
			cs = cs+1
			break

		r = numpy.ones(self.num_joints)		# generate random sample
		for i in range(0, self.num_joints):
			r[i] = 2*math.pi*(random.random()-0.5)
		if self.is_state_valid(r) == False:
			continue
		print('random sample', r)
		l = []
		for j in numpy.arange(len(tree)):	# find the min lenth from sample to tree
			l.append( numpy.linalg.norm((r-tree[j])))
		minl = min(l)
		k = l.index(minl)			# the node of min len
		selectq =tree[k]
		
		v = r - selectq				# generate new branch
		c = numpy.linalg.norm(r - selectq)
		branch = v/c *0.1 + selectq
		
		if self.is_segment_valid(branch, selectq) == True:
			
			xin = branch
			tree.append(xin)
			parent.append(k+1)
			cs = cs+1
		print('get a forward branch', xin)	
		# start generate backforward tree
		
		l = []
		for j in numpy.arange(len(treeb)):	# find the min lenth from sample to tree
			l.append(numpy.linalg.norm((r-treeb[j])))
		minl = min(l)
		k = l.index(minl)			# the node of min len
		selectqb =treeb[k]
		
		v = r - selectqb			# generate new branch
		c = numpy.linalg.norm(r - selectqb)
		branch = v/c *0.1 + selectqb
		
		if self.is_segment_valid(branch, selectqb) == True:
			
			xinb = branch
			treeb.append(xinb)
			parentb.append(k+1)
			cd = cd+1
		print('get a backward branch', xinb)
			

		        
		if numpy.linalg.norm(xinb-xin)<left:
			left = numpy.linalg.norm(xinb-xin)
		print('road left                                 ',left)

#	print('tree',tree)
#	print('parent',parent)
#	print('lenth of parent',len(parent))
#	print('tree back',treeb)
#	print('parent back',parentb)
#	print('lenth of parent back',len(parentb))
	print('get forward path')
# forward path
	treepath = tree[-1]
	parentpath = parent[-1]
	
        path = []
	path.append(treepath)
	next = treepath
	while numpy.linalg.norm(next- currentq)> 0.01:
#		print('lenth of parent',len(parent))
#		print('next is',next)
#		print('parent is', parentpath)
		next = tree[parentpath-1]
		parentpath = parent[parentpath-1]
		path.append(next)
#	print('the first part of path is ',path)

	print('get backward path')
# backward path
	treepathb = treeb[-1]
	parentpathb = parentb[-1]
        pathb = []
	pathb.append(treepathb)
	nextb = treepathb
	while numpy.linalg.norm(nextb - goalq)> 0.01:
		
		nextb = treeb[parentpathb-1]
		parentpathb = parentb[parentpathb-1]
		pathb.append(nextb)
	print('second part of path is ', pathb)
# reverse sequense of forward path
	inversepath = []
	l = len(path)
	for i in numpy.arange(l):
		inversepath.append(path[l-1-i])

# add the pathb to 'inversepath'
	for i in pathb:
		inversepath.append(i)		
#	print('the whole path is', inversepath)

	#print('111111111111111111111111111111111111111111111111111111')
	dian = len(inversepath)
	
	p1 = currentq 

	steplist = []
	steplist.append(p1)
	print('dian',dian)
	print('end',goalq)
	seg = 5
	span = numpy.ceil(dian/seg)
	n = int(span)
	#print('222222222222222222222222222222222222222222222222222222')
	j = 0
	p = inversepath[n]
	while n<(dian):
		print(dian,span,n,j)		
		#print('start node is', p1)
		#print('end node is', p)
		if self.is_segment_valid(p, p1) == True:
			#print('if no collision')
			fangxiang = p - p1
			changdu = numpy.linalg.norm(fangxiang)
			if changdu == 0.0: 
				steplist.append(p1)
				break # modified by hsbnd
				
			num = changdu/0.025
			num = numpy.ceil(num)
			step = fangxiang/num
			#print('step is',step)
			for i in numpy.arange(0, num):#num+1
				p1= p1 +step
				steplist.append(p1)
			p1 = p
			n = n + int(span)
			
			if n>=dian:
				if j==1: break
				n = dian-1
				#print('last',n)
				j = j+1
				
			p = inversepath[n]
			fangxiang = p - p1
			changdu = numpy.linalg.norm(fangxiang)
			
		if changdu == 0.0: 
			break # modified by hsbnd
			
		elif self.is_segment_valid(p, p1) == False:
			#print('collision')
			n = n -1
			p = inversepath[n]
	
			
		#print(n)
	print(steplist[-1])
	
	#print('333333333333333333333333333333333333333333333333333333')
	
	trajectory= JointTrajectory()
	trajectory.joint_names = self.joint_names
	for i in range(0, len(steplist)):
		a =JointTrajectoryPoint()
		a.positions = steplist[i]
		trajectory.points.append(a)

	self.pub.publish(trajectory)






        ######################################################

    def is_segment_valid(self, qg, qc):	
	fangxiang = qg - qc
	changdu = numpy.linalg.norm(fangxiang)
#	print('changdu',changdu)
	if changdu == 0.0:
		return True
	num = changdu/0.06# 0.0001
	
	num = numpy.ceil(num)
	step = fangxiang/num
	#steplist = []
	#steplist.append(qc)
	qi = qc
	for i in numpy.arange(0, num):#num+1
		qi= qi + step
		if self.is_state_valid(qi) is False:
			return False
			break		#steplist.append(qc)
       	return True	
    

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

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


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

