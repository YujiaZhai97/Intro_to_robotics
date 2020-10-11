#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from assignment0.msg import TwoInt
sum12= Int16()

def callback(num):
    global sum12
    sum12=num.num1+num.num2
    rospy.loginfo(sum12)
    
def adder():
    rospy.init_node('adder', anonymous=True)
    

    pub=rospy.Publisher('/sum', Int16, queue_size=10)
    sub=rospy.Subscriber('/numbers', TwoInt, callback)

    rate=rospy.Rate(10)
    
    while not rospy.is_shutdown():
              pub.publish(sum12)
              
              rate.sleep()
if __name__ == '__main__':
           adder()

