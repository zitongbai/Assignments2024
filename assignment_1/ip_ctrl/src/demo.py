#! /bin/python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import numpy as np

class DemoCtrl:
    """A demo class
    This class shows how to receive joint states and publish 
    control commands (force in horizontal direction)
    """
    def __init__(self) -> None:
        rospy.init_node('demo_ctrl', anonymous=True)
        
        # TODO: 请填写subscriber的topic name
        joint_states_topic_name = None
        assert joint_states_topic_name is not None, 'Please fill in the topic name' # 填写后可以删除这行
        self.joint_states_sub = rospy.Subscriber(joint_states_topic_name, JointState, self.joint_states_callback)
        
        # TODO: 请填写publisher的topic name
        joint_ctrl_topic_name = None
        assert joint_ctrl_topic_name is not None, 'Please fill in the topic name' # 填写后可以删除这行
        self.joint_ctrl_pub = rospy.Publisher(joint_ctrl_topic_name, Float64, queue_size=10)
        
        self.run()
        
    def joint_states_callback(self, msg:JointState):
        joint_names = msg.name
        joint_positions = msg.position
        joint_velocities = msg.velocity
        print('joint_names:', joint_names, 'joint_positions:', joint_positions, 'joint_velocities:', joint_velocities)
        
    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            
            ctrl_msg = Float64()
            ctrl_msg.data = np.sin(rospy.Time.now().to_sec()) # horizontal force
            
            self.joint_ctrl_pub.publish(ctrl_msg)
            rate.sleep()

if __name__ == '__main__':
    demo_ctrl = DemoCtrl()
    