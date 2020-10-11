#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from assignment0.msg import TwoInt
def generator():
    pub = rospy.Publisher('/numbers', TwoInt, queue_size=10)
    rospy.init_node('generator', anonymous=True)
    rate = rospy.Rate(10)
    num=TwoInt()
    while not rospy.is_shutdown():
              
              num.num1 = random.randint(0,100)
              num.num2= random.randint(0,100)
              rospy.loginfo(num)
              pub.publish(num)
              rate.sleep()
if __name__ == '__main__':
    try:
        generator()
    except rospy.ROSInterruptException:
        pass

