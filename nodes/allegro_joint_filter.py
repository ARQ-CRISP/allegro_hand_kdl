#! /usr/bin/env python
from __future__ import division, print_function, absolute_import
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from collections import deque, namedtuple
from scipy.signal import butter, filtfilt

joint_state = namedtuple('joint_state', ['position', 'velocity', 'effort'])

class Allegro_State_Filter(object):
    
    ALLEGRO_SOURCE = '/allegroHand_0/joint_states'
    ALLEGRO_FILTERED = '/allegroHand_0/joint_states_filtered'

    def __init__(self, sample_len, min_len=1):
        self.queue = deque(maxlen=sample_len)
        self.current_state = None
        self.min_len = min_len
        rospy.init_node('allegro_filter')
        rospy.loginfo('Initialising Filter Node... Done!')
        self.subscriber = rospy.Subscriber(self.ALLEGRO_SOURCE, JointState, self.On_State_Received, )
        self.publisher = rospy.Publisher(self.ALLEGRO_FILTERED, JointState, queue_size=10)
        rospy.loginfo('Subscribing to topics... Done!')
        
    def On_State_Received(self, msg):
        state = joint_state(np.asarray(msg.position), np.asarray(msg.velocity), np.asarray(msg.effort))
        self.queue.append(state)

        if len(self.queue) > self.min_len:
            filtered_state = JointState()
            filtered_state.header = msg.header
            filtered_state.name = msg.name
            filtered_state.position, filtered_state.velocity, filtered_state.effort = self.filter_data()
            self.current_state = joint_state(filtered_state.position, filtered_state.velocity, filtered_state.effort)
            self.publisher.publish(filtered_state)
            # print(self.current_state.position[9])

    def filter_data(self):
        """returns filtered (positions, velocity, effort) of the allegro hand"""
        raise NotImplementedError('filter_data function should be implemented based on the filter logic!')

class Allegro_Avg_Filter(Allegro_State_Filter):

    def filter_data(self):
        return np.mean([qel.position for qel in self.queue], axis=0).tolist(), \
        np.mean([qel.velocity for qel in self.queue], axis=0).tolist(), \
            np.mean([qel.effort for qel in self.queue], axis=0).tolist()

class Allegro_Median_Filter(Allegro_State_Filter):
    
    def filter_data(self):
        return np.median([qel.position for qel in self.queue], axis=0).tolist(), \
        np.median([qel.velocity for qel in self.queue], axis=0).tolist(), \
            np.median([qel.effort for qel in self.queue], axis=0).tolist()

class Allegro_ButterAvg_Filter(Allegro_State_Filter):
    
    def __init__(self, sample_len, fs = 330.0, cutoff = 5, order=2, avg_len=25):
        self._fs = fs
        self._cutoff = cutoff
        self._order = order
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        self.avg_len = avg_len
        super(Allegro_ButterAvg_Filter, self).__init__(sample_len, 5)

    def filter_data(self):
        pos_hist = [qel.position for qel in self.queue]
        vel_hist = [qel.velocity for qel in self.queue]
        effort_hist = [qel.effort for qel in self.queue]
        return filtfilt(self.b, self.a, pos_hist, axis=0, padlen=5)[-self.avg_len:].mean(axis=0).tolist(), \
            filtfilt(self.b, self.a, vel_hist, axis=0, padlen=5)[-self.avg_len:].mean(axis=0).tolist(), \
                filtfilt(self.b, self.a, effort_hist, axis=0, padlen=5)[-self.avg_len:].mean(axis=0).tolist(), \
        


if __name__ == "__main__":

    # allegro_filter = Allegro_Avg_Filter(100)
    # allegro_filter = Allegro_Median_Filter(100)
    allegro_filter = Allegro_ButterAvg_Filter(100)

    rospy.spin()

