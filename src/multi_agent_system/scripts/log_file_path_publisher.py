#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publish_log_file_path():
    rospy.init_node('log_file_path_publisher', anonymous=True)
    pub = rospy.Publisher('/log_file_path', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    log_file_path = rospy.get_param('~log_file_path', '')

    while not rospy.is_shutdown():
        pub.publish(log_file_path)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_log_file_path()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
