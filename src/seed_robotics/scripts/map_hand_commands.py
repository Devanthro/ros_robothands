#!/usr/bin/env python3.7

import rospy
from sensor_msgs.msg import JointState
from seed_robotics.msg import JointSetSpeedPos, JointListSetSpeedPos

# Initialize a ROS Node
rospy.init_node('operator_joint_state_listener', anonymous=True)

# Initialize a Publisher to the 'R_speed_position' Topic
pub = rospy.Publisher('R_speed_position', JointListSetSpeedPos, queue_size=1)

# Define joint configurations and mapping from left to right hand
joint_configs = {
    'l_w_rotation': {'input_range': (-1.57, 1.57), 'output_range': (0, 4095)},
    'l_w_adduction': {'input_range': (-0.785, 0.785), 'output_range': (0, 4095)},
    'l_w_flexion': {'input_range': (-0.785, 0.785), 'output_range': (0, 4095)},
    'l_th_adduction': {'input_range': (0, 1), 'output_range': (0, 4095)},
    'l_th_flexion': {'input_range': (0, 1), 'output_range': (0, 4095)},
    'l_ix_flexion': {'input_range': (0, 1), 'output_range': (0, 4095)},
    'l_middle_flexion': {'input_range': (0, 1), 'output_range': (0, 4095)},
    'l_ring_ltl_flexion': {'input_range': (0, 1), 'output_range': (0, 4095)},
}

# Map left-hand joint names to right-hand joint names
left_to_right_mapping = {
    'l_w_rotation': 'r_w_rotation',
    'l_w_adduction': 'r_w_adduction',
    'l_w_flexion': 'r_w_flexion',
    'l_th_adduction': 'r_th_adduction',
    'l_th_flexion': 'r_th_flexion',
    'l_ix_flexion': 'r_ix_flexion',
    'l_middle_flexion': 'r_middle_flexion',
    'l_ring_ltl_flexion': 'r_ring_ltl_flexion',
}

# Helper function for scaling values
def scale_value(value, input_range, output_range):
    input_min, input_max = input_range
    output_min, output_max = output_range
    scaled = ((value - input_min) / (input_max - input_min)) * (output_max - output_min) + output_min
    return max(min(int(scaled), output_max), output_min)

# Callback function to process incoming JointState messages
def joint_state_callback(msg):
    rospy.loginfo_throttle(1, f"Received JointState: {msg}")
    joint_message_list = []

    for name, position in zip(msg.name, msg.position):
        if name in joint_configs:
            config = joint_configs[name]
            target_pos = scale_value(position, config['input_range'], config['output_range'])
            try:
                if name == "l_th_adduction":
                    target_pos *= 2
                    target_pos = max(min(target_pos, 4095),0) 
            except:
                pass

            # Map the left-hand joint to its corresponding right-hand joint
            right_hand_name = left_to_right_mapping.get(name)
            if not right_hand_name:
                continue

            joint_message = JointSetSpeedPos(
                name=right_hand_name,
                target_pos=target_pos,
                target_speed=0  # Maximum speed
            )
            joint_message_list.append(joint_message)

    # Publish the JointListSetSpeedPos message
    if joint_message_list:
        message_to_send = JointListSetSpeedPos(joints=joint_message_list)
        pub.publish(message_to_send)
        rospy.loginfo_throttle(1, f"Published Command: {message_to_send}")

# Subscribe to the 'filtered_joint_states' Topic for left-hand commands
rospy.Subscriber('filtered_joint_states', JointState, joint_state_callback)

# Keep the script running
rospy.loginfo("Operator Joint State Listener is running...")
rospy.spin()


# import rospy
# from sensor_msgs.msg import JointState
# from seed_robotics.msg import JointSetSpeedPos, JointListSetSpeedPos

# # Initialize a ROS Node
# rospy.init_node('operator_joint_state_listener', anonymous=True)

# # Initialize a Publisher to the 'R_speed_position' Topic
# pub = rospy.Publisher('R_speed_position', JointListSetSpeedPos, queue_size=1)

# # Define the mapping between the operator's joint state names and the robot's joint names
# # 'r_w_rotation','r_w_adduction','r_w_flexion','r_th_adduction','r_th_flexion','r_ix_flexion','r_middle_flexion','r_ring_ltl_flexion'
# joint_mapping = {
#     'finger_1_joint': 'r_ix_flexion',
#     'finger_2_joint': 'r_middle_flexion',
#     'finger_3_joint': 'r_ring_ltl_flexion',
#     'thumb_joint': 'r_th_flexion',
#     # Add other mappings as needed
# }
# r_hand_joints = ['r_w_rotation','r_w_adduction','r_w_flexion','r_th_adduction','r_th_flexion','r_ix_flexion','r_middle_flexion','r_ring_ltl_flexion']

# # Callback function to process incoming JointState messages
# def joint_state_callback(msg):
#     rospy.loginfo_throttle(1,f"got {msg}")
#     joint_message_list = []
#     for name, position in zip(msg.name, msg.position):
#         if name in r_hand_joints:
#             joint_message = JointSetSpeedPos()
#             joint_message.name = name #joint_mapping[name]
#             joint_message.target_pos = int(position*4095)  # Scale as needed
#             if joint_message.target_pos > 4095:
#                 joint_message.target_pos = 4095
#             elif joint_message.target_pos < 0:
#                 joint_message.target_pos = 0
#             joint_message.target_speed = 0  # Maximum speed
#             joint_message_list.append(joint_message)

#     # Publish the JointListSetSpeedPos message
#     if joint_message_list:
#         message_to_send = JointListSetSpeedPos()
#         message_to_send.joints = joint_message_list
#         # pub.publish(message_to_send)
#         rospy.loginfo_throttle(1,f"Command: {message_to_send}")

# # Subscribe to the '/operator/alice/joint_state' Topic
# rospy.Subscriber('filtered_joint_states', JointState, joint_state_callback)

# # Keep the script running
# rospy.loginfo("Operator Joint State Listener is running...")
# rospy.spin()
