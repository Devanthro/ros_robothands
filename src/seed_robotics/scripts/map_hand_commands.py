#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState
from seed_robotics.msg import JointSetSpeedPos, JointListSetSpeedPos

# Initialize a ROS Node
rospy.init_node('operator_joint_state_listener', anonymous=True)

# Initialize a Publisher to the 'R_speed_position' Topic
pub_L = rospy.Publisher('L_speed_position', JointListSetSpeedPos, queue_size=1)
pub_R = rospy.Publisher('R_speed_position', JointListSetSpeedPos, queue_size=1)

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
    'r_w_rotation': {'input_range': (-1.57, 1.57), 'output_range': (0, 4095)},
    'r_w_adduction': {'input_range': (-0.785, 0.785), 'output_range': (0, 4095)},
    'r_w_flexion': {'input_range': (-0.785, 0.785), 'output_range': (0, 4095)},
    'r_th_adduction': {'input_range': (0, 1), 'output_range': (0, 4095)},
    'r_th_flexion': {'input_range': (0, 1), 'output_range': (0, 4095)},
    'r_ix_flexion': {'input_range': (0, 1), 'output_range': (0, 4095)},
    'r_middle_flexion': {'input_range': (0, 1), 'output_range': (0, 4095)},
    'r_ring_ltl_flexion': {'input_range': (0, 1), 'output_range': (0, 4095)},
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
    joint_message_list_R = []
    joint_message_list_L = []

    for name, position in zip(msg.name, msg.position):
        if name in joint_configs:
            config = joint_configs[name]
            target_pos = scale_value(position, config['input_range'], config['output_range'])
            try:
                if name == "l_th_adduction" or name == "r_th_adduction":
                    target_pos *= 2
                    target_pos = max(min(target_pos, 4095),0) 
            except:
                pass

            # # Map the left-hand joint to its corresponding right-hand joint
            # right_hand_name = left_to_right_mapping.get(name)
            # if not right_hand_name:
            #     continue

            joint_message = JointSetSpeedPos(
                name=name,
                # name=right_hand_name,
                target_pos=target_pos,
                target_speed=0  # Maximum speed
            )
            if name.startswith('l_'):
                joint_message_list_L.append(joint_message)
            elif name.startswith('r_'):
                joint_message_list_R.append(joint_message)

    # Publish the JointListSetSpeedPos message
    for joint_message_list, pub in zip([joint_message_list_L, joint_message_list_R], [pub_L, pub_R]):
        if joint_message_list:
            message_to_send = JointListSetSpeedPos(joints=joint_message_list)
            pub.publish(message_to_send)
            rospy.loginfo_throttle(1, f"Published Command: {message_to_send}")

# Subscribe to the 'filtered_joint_states' Topic for left-hand commands
topic_name = 'filtered_joint_states'
rospy.Subscriber(topic_name, JointState, joint_state_callback)

# Keep the script running
rospy.loginfo(f"Operator Joint State Listener is running. Subscribed to {topic_name}...")
rospy.spin()
