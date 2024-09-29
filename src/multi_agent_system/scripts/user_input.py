#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def user_input_node():
    rospy.init_node('user_input', anonymous=True)
    pub = rospy.Publisher('/user_command', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    print("Multi-Agent System is running. Type your commands and press Enter.")
    print("Type 'exit' to quit.")

    while not rospy.is_shutdown():
        user_input = input("Enter command: ")
        if user_input.lower() == 'exit':
            break
        pub.publish(user_input)
        rate.sleep()

if __name__ == '__main__':
    try:
        user_input_node()
    except rospy.ROSInterruptException:
        pass
