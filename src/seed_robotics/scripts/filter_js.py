#!/usr/bin/env python3.7


import rospy
from sensor_msgs.msg import JointState

rospy.init_node('filter_joint_state')



global filtered_msg

def callback(msg):
    global filtered_msg
    # Check if the desired joint name exists in the message
    if 'l_th_adduction' in msg.name:
        # print("got a msg")
        # Extract the index of the joint
        index = msg.name.index('l_th_adduction')
        # Create a new JointState message with only the desired joint's data
        filtered_msg = JointState()
        filtered_msg.header = msg.header
        filtered_msg.name = msg.name
        filtered_msg.position = msg.position
        if msg.velocity:
            filtered_msg.velocity = msg.velocity
        if msg.effort:
            filtered_msg.effort = msg.effort
        # Publish the filtered message
        
        # rospy.sleep(rate)

if __name__ == '__main__':
    
    global filtered_msg
    filtered_msg = JointState()
    # Publisher for the filtered messages
    pub = rospy.Publisher('filtered_joint_states', JointState, queue_size=1)
    
    # Subscriber to the original JointState topic
    rospy.Subscriber('/operator/alice/joint_state', JointState, callback)
    

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        pub.publish(filtered_msg)
        rate.sleep()
