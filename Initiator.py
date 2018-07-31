import rospy
from PrimaryHandler import *

if __name__ == "__main__":
    rospy.init_node('Demo', anonymous=True)
    miro1 = SecondaryInterface('rob01', 1, 6.283185307/2)
    miro1.sensorinterrupt()
