#!/usr/bin/env python

from __future__ import print_function

import string

import rospy
import rosbag

from sensor_msgs.msg import JointState

class CapturePoses:
    last_state_ = None # last joint states

    def __init__(self):
        rospy.init_node('capture_calibration_poses')
        rospy.Subscriber("joint_states", JointState, self.stateCb)

        # bag to write data to
        bag = rosbag.Bag('calibration_poses.bag', 'w')

        # put samples in
        while not rospy.is_shutdown():
            print('Move arm/head to a new sample position:  (type "exit" to quit and save data)')
            resp = raw_input('press <enter>')
            if string.upper(resp) == 'EXIT':
                break
            else:
                if self.last_state_ == None:
                    print('Cannot save state')
                    continue
                # save a pose
                bag.write('calibration_joint_states', self.last_state_)
        bag.close()
            

    def stateCb(self, msg):
        """ Callback for joint_states messages """
        self.last_state_ = msg

if __name__=='__main__':
    CapturePoses()
