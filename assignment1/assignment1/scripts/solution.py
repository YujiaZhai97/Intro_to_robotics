#!/usr/bin/env python
import rospy
import numpy
import tf
import tf2_ros
import geometry_msgs.msg

from numpy import linalg as LA



def publish_transforms():

    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_frame"
    t1.child_frame_id = "object_frame"
    q1 = tf.transformations.quaternion_from_euler(0.64, 0.64, 0.0)
    transf1 = numpy.dot(tf.transformations.quaternion_matrix(q1),
                   tf.transformations.translation_matrix((1.5, 0.8, 0.0))
                  )
   
    tr1 = tf.transformations.translation_from_matrix(transf1)
    t1.transform.translation.x = tr1[0]
    t1.transform.translation.y = tr1[1]
    t1.transform.translation.z = tr1[2]
    t1.transform.rotation.x = q1[0]
    t1.transform.rotation.y = q1[1]
    t1.transform.rotation.z = q1[2]
    t1.transform.rotation.w = q1[3]
    br.sendTransform(t1)


    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_frame"
    t2.child_frame_id = "robot_frame"

    transf2 = numpy.dot(tf.transformations.rotation_matrix(1.5,(0, 1, 0)),
                   tf.transformations.translation_matrix((0.0, 0.0, -2.0))
                  )  

    tr2 = tf.transformations.translation_from_matrix(transf2)
    t2.transform.translation.x = tr2[0]
    t2.transform.translation.y = tr2[1]
    t2.transform.translation.z = tr2[2]
    q2 = tf.transformations.quaternion_from_matrix(transf2)
    t2.transform.rotation.x = q2[0]
    t2.transform.rotation.y = q2[1] 
    t2.transform.rotation.z = q2[2]
    t2.transform.rotation.w = q2[3]
    br.sendTransform(t2)

 
    
    

    t3 = geometry_msgs.msg.TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "robot_frame"
    t3.child_frame_id = "camera_frame"
    t3.transform.translation.x = 0.3
    t3.transform.translation.y = 0.0
    t3.transform.translation.z = 0.3

   
    bo = numpy.array([tr1[0], tr1[1], tr1[2], 1])
   
    
    tr3 = tf.transformations.translation_matrix((0.3, 0.0, 0.3))
    tr3_inverse = tf.transformations.translation_matrix((-0.3, 0.0, -0.3))
    transf2_inverse = tf.transformations.inverse_matrix(transf2)
    guocheng = numpy.dot(transf2_inverse, bo)
    co = numpy.dot(tr3_inverse, guocheng)

 
    v_co = co
    vco = numpy.array([v_co[0], v_co[1], v_co[2]])

    cx = numpy.array([1.0, 0.0, 0.0])

        
    vcolen = LA.norm(vco)
    cxlen = LA.norm(cx)
    
    jiao = numpy.arccos(numpy.dot(cx, vco)/(vcolen*cxlen))
    
    zhou = numpy.cross(cx, vco)
    

 
    q3 = tf.transformations.quaternion_about_axis(jiao, zhou)
    t3.transform.rotation.x = q3[0]
    t3.transform.rotation.y = q3[1] 
    t3.transform.rotation.z = q3[2]
    t3.transform.rotation.w = q3[3]
    br.sendTransform(t3)


    

if __name__ == '__main__':
    rospy.init_node('solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.1)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.1)
