# SK & bourne

import os
import glob
import rosbag
import logging
import pickle
from smach_based_introspection_framework._constant import (
    anomaly_label_file,
    RECOVERY_DEMONSTRATED_BY_HUMAN_TAG,
)
import itertools
from Anomaly import Anomaly
from Skill import Skill
from Demonstration import Demonstration
import ipdb
import numpy as np
import string
from math import floor
ADAPTATION_RATIO = 1
EXEC_TIME_RATIO = 1
class ExperimentRecord(object):
    def __init__(self, folder_path):
        self.folder_path = folder_path
        logger = logging.getLogger('ExpRecordOf...%s'%str(folder_path)[-20:])
        self.logger = logger
        

    @property
    def tag_ranges(self):
        if hasattr(self, "_tag_ranges"):
            return self._tag_ranges

        last_tag = None
        last_tag_starts_at = None
        ranges = []
        for count, (topic, msg, record_time) in enumerate(self.rosbag.read_messages('/tag_multimodal')):
            cur_tag = int(msg.tag)
            if last_tag is None:
                last_tag = cur_tag
                last_tag_starts_at = msg.header.stamp

            if cur_tag != last_tag:
                ranges.append((last_tag, (last_tag_starts_at, msg.header.stamp)))
                last_tag = cur_tag
                last_tag_starts_at = msg.header.stamp
        ranges.append((last_tag, (last_tag_starts_at, msg.header.stamp)))

        self._tag_ranges = ranges
        return self._tag_ranges

# ================================================================================================

    @property
    def get_anomaly_label_to_indexes(self):
        anomaly_labels_indexes_list = []
        anomaly_labels_path = os.path.join(self.folder_path,'anomaly_labels.txt')
        if not os.path.exists(anomaly_labels_path):
            return 'No_anomaly_label'

        with open(anomaly_labels_path,"r") as f:
            anomaly_labels = f.read()
        anomaly_labels_list = string.split(anomaly_labels,'\n')
        for anomaly_label in anomaly_labels_list:
            if anomaly_label == 'human_collision':
                anomaly_label = -2000
            elif anomaly_label == 'tool_collision':
                anomaly_label = -2001
            elif anomaly_label == 'object_slip':
                anomaly_label = -2002
            elif anomaly_label == 'wall_collision':
                anomaly_label = -2003
            elif anomaly_label == 'no_object':
                anomaly_label = -2004
            elif anomaly_label == 'human_collision_with_object':
                anomaly_label = -2000
            elif anomaly_label == 'false_positive':
                return 'false_positive'
            elif anomaly_label == 'Unlabeled':
                return 'Unlabeled'
            anomaly_labels_indexes_list.append(anomaly_label)
        anomaly_labels_indexes_list.remove('')
        # reverse for pop out
        anomaly_labels_indexes_list.reverse()
        return anomaly_labels_indexes_list
    


    @property
    def extract_episode_tuples_for_q_table(self):

        state = np.array((0,0))
        next_state = np.array((0,0))
        tuple_next_phase_recorded = False
        consider_anomaly_tag = False
        end_time_on_tag_0 = False
        norminal = 0
        last_tag = None
        exp_tuples_list = []
        exp_tuples_with_exec_time_list = []
        anomaly_labels_indexes_list = self.get_anomaly_label_to_indexes
        if anomaly_labels_indexes_list == 'false_positive' or  anomaly_labels_indexes_list == 'Unlabeled':
            return anomaly_labels_indexes_list

        for count, (topic, msg, record_time) in enumerate(self.rosbag.read_messages('/tag_multimodal')):

            msgString = str(msg)
            msgList = string.split(msgString, '\n')
            rowmsg = {}
            for nameValuePair in msgList:
                splitPair = string.split(nameValuePair, ':')
                if splitPair[0] == 'tag':
                    splitPair[1] = string.replace(splitPair[1],' ','')
                    rowmsg[splitPair[0]] = int(splitPair[1])
                else:
                    rowmsg[splitPair[0]] = splitPair[1]
            if rowmsg['tag'] == -4:
                a = 1
            if end_time_on_tag_0 == True and rowmsg['tag'] == 0:
                end_time = record_time.secs
                end_time_on_tag_0 = False
                # error
            if rowmsg['tag'] !=0 and last_tag != rowmsg['tag']:
                if consider_anomaly_tag==False and rowmsg['tag'] < 0:
                    continue
                # record the state.phase
                if rowmsg['tag'] > 0 and tuple_next_phase_recorded == False:
                    act = rowmsg['tag']
                    next_state = np.array((0,0))
                    next_state[0] = rowmsg['tag']
                    start_time = record_time.secs
                    tuple_next_phase_recorded = True
                    consider_anomaly_tag = True
                    end_time_on_tag_0 = True
                    last_tag =  rowmsg['tag']
                    continue
                else :
                    # record the state.anomaly_condition
                    if rowmsg['tag'] >0:
                        # no anomaly happened when executing
                        next_state[1] = norminal
                    elif rowmsg['tag'] < 0:
                        # Anomaly happened when executing
                        if  anomaly_labels_indexes_list =='No_anomaly_label':
                            # print('There is -1 in /tag_multimodal topic, but no anomaly label according, gonna skip this rosbag.')
                            return 'No_anomaly_label_with_tag_-1'
                        try:
                            next_state[1] =anomaly_labels_indexes_list.pop()
                        except ValueError :
                            return "unknown_label"
                        consider_anomaly_tag = False
                        last_tag = rowmsg['tag']
                    exec_time = floor(end_time - start_time)
                    tuple_s = tuple(state)
                    tuple_n_s = tuple(next_state)
                    exp_tuple = (tuple_s,act,tuple_n_s,exec_time)
                    exp_tuples_list.append(exp_tuple)
                    state = np.array((0,0))
                    state = next_state
                    if np.all(next_state == np.array((9,0))):
                        # Reset an episode
                        state = np.array((0,0))
                    tuple_next_phase_recorded = False
                    

        tuple_s = tuple(state)
        tuple_n_s = tuple(next_state)
        
        # Do not count the skill 9 exec time
        exec_time = 2
        try :
            exp_tuple = (tuple_s,act,tuple_n_s,exec_time)
        except UnboundLocalError as err:
            return 'No_nominal_tag_is_recorded'
        if next_state[0] != 9:
            return 'no_tag_9'
        exp_tuples_list.append(exp_tuple)
        exp_tuples_with_exec_time_list = self.get_states_with_time(exp_tuples_list)
        return exp_tuples_with_exec_time_list

    # ================================================================================================

    def get_states_with_time(self, exp_tuples_list):
        state_action_time = 0
        exp_tuples_with_exec_time_list = []
        for e_tuple in exp_tuples_list:
            state, act, next_state, exec_time = e_tuple
            state = (state[0],state[1],state_action_time)
            next_state =  (next_state[0],next_state[1],exec_time)
            state_action_time = exec_time
            new_tuple = (state, act, next_state)
            exp_tuples_with_exec_time_list.append(new_tuple)
        return exp_tuples_with_exec_time_list


    @property
    def anomaly_signal_times(self):
        if hasattr(self, "_anomaly_signal_times"):
            return self._anomaly_signal_times

        signals = []
        for topic_name, msg, gen_time in self.rosbag.read_messages(topics=["/anomaly_detection_signal"]):
            if len(signals) == 0:
                signals.append(msg.stamp)
            else:
                time_diff = (msg.stamp-prev_msg.stamp).to_sec()
                if time_diff > 1:
                    signals.append(msg.stamp)
                elif time_diff < 0:
                    raise Exception("Weird error: anomaly signal messages read from rosbag are not in time-increasing order")
            prev_msg = msg

        self._anomaly_signal_times = signals
        return self._anomaly_signal_times

    @property
    def rosbag(self):
        if hasattr(self, "_rosbag"):
            return self._rosbag
        paths = glob.glob(os.path.join(
            self.folder_path,
            '*.bag',
        ))
        bags = [rosbag.Bag(p) for p in paths] 
        if len(bags) == 0:
            raise Exception("No rosbag in %s"%self.folder_path)
        elif len(bags) > 1:
            raise Exception("More than 1 rosbags in %s"%self.folder_path)
        self._rosbag = bags[0]
        return self._rosbag

    @property
    def unsuccessful_tag_ranges(self):
        if hasattr(self, "_unsuccessful_tag_ranges"):
            return self._unsuccessful_tag_ranges

        unsucc_tag_ranges = []
        for idx, (tag, (start_time, end_time)) in enumerate(self.tag_ranges):
            try:
                next_tag = 0
                forward = 1
                while next_tag == 0:
                    next_tag = self.tag_ranges[idx+forward][0]
                    forward += 1
            except IndexError:
                break
            if tag > 0 and next_tag < 0:
                unsucc_tag_ranges.append((tag, (start_time, end_time)))

        self._unsuccessful_tag_ranges = unsucc_tag_ranges
        return self._unsuccessful_tag_ranges

    @property
    def successful_tag_ranges(self):
        if hasattr(self, "_successful_tag_ranges"):
            return self._successful_tag_ranges

        succ_tag_ranges = []
        for idx, (tag, (start_time, end_time)) in enumerate(self.tag_ranges):
            try:
                next_tag = 0
                forward = 1
                while next_tag == 0:
                    next_tag = self.tag_ranges[idx+forward][0]
                    forward += 1
            except IndexError:
                next_tag = 0
            if tag > 0 and next_tag >= 0:
                succ_tag_ranges.append((tag, (start_time, end_time)))

        self._successful_tag_ranges = succ_tag_ranges
        return self._successful_tag_ranges

    @property
    def anomaly_signals(self):
        if hasattr(self, '_anomaly_signals'):
            return self._anomaly_signals

        signals = self.anomaly_signal_times

        if len(signals) > len(self.anomaly_labels):
            raise Exception("anomaly signals amount, %s, does not match anomaly labels, %s."%(len(signals), len(self.anomaly_labels)))
        elif len(signals) != len(self.unsuccessful_tag_ranges):
            raise Exception("anomaly signals amount, %s, does not match unsuccessful skill amount, %s."%(len(signals), len(self.unsuccessful_tag_ranges)))
        elif len(signals) < len(self.anomaly_labels):
            self.logger.warn("anomaly signals amount, %s, does not match anomaly labels, %s. But I will try to pair them in reverse order."%(len(signals), len(self.anomaly_labels)))
            labels = self.anomaly_labels[-len(signals):]
        else:
            labels = self.anomaly_labels
        
        self._anomaly_signals = zip(labels, signals)
        return self._anomaly_signals

    @property
    def anomaly_labels(self):
        if hasattr(self, '_anomaly_labels'):
            return self._anomaly_labels

        txt_path = os.path.join(self.folder_path, anomaly_label_file)
        if not os.path.isfile(txt_path):
            return []

        lines = open(txt_path, 'r').readlines()
        labels = [i.strip() for i in lines]
        labels = [i for i in labels if i != ""]
        self._anomaly_labels = labels
        return self._anomaly_labels

    @property
    def list_of_anomalies(self):
        if hasattr(self, "_list_of_anomalies"):
            return self._list_of_anomalies

        signals = self.anomaly_signal_times
        if len(signals) != len(self.anomaly_labels):
            raise Exception("anomaly signals amount, %s, does not match anomaly labels, %s."%(len(signals), len(self.anomaly_labels)))
        elif len(signals) != len(self.unsuccessful_tag_ranges):
            raise Exception("anomaly signals amount, %s, does not match unsuccessful skill amount, %s."%(len(signals), len(self.unsuccessful_tag_ranges)))
        else:
            labels = self.anomaly_labels

        list_of_anomalies = []
        for label, signal in itertools.izip(labels, signals):
            anomaly_instance = Anomaly()
            anomaly_instance.label = label
            anomaly_instance.time = signal

            unsucc_skill_idx = None
            for idx, (tag, (st, et)) in enumerate(self.unsuccessful_tag_ranges):
                if st > signal:
                    break
                unsucc_skill_idx = idx

            if unsucc_skill_idx is None:
                raise Exception()
            tup = self.unsuccessful_tag_ranges[unsucc_skill_idx]
            skill = Skill()
            skill.tag = str(tup[0])
            skill.start_time = tup[1][0]
            skill.end_time = tup[1][1]
            anomaly_instance.skill_belonged_to = skill
            list_of_anomalies.append(anomaly_instance)

        self._list_of_anomalies = list_of_anomalies
        return list_of_anomalies

    @property
    def list_of_demonstrations(self):
        if hasattr(self, "_list_of_demonstrations"):
            return self._list_of_demonstrations

        list_of_demonstrations = []
        for idx, (tag, (start_time, end_time)) in enumerate(self.tag_ranges):
            if tag != RECOVERY_DEMONSTRATED_BY_HUMAN_TAG:
                continue

            demonstration = Demonstration()
            demonstration.start_time = start_time
            demonstration.end_time = end_time

            for anomaly in self.list_of_anomalies:
                if demonstration.start_time > anomaly.time:
                    demonstration.targeted_anomaly = anomaly
                else:
                    break

            for topic, msg, record_time in self.rosbag.read_messages('/observation/goal_vector', end_time=end_time):
                demonstration.original_goal = msg.goal_vector

            list_of_demonstrations.append(demonstration)

        self._list_of_demonstrations = list_of_demonstrations
        return list_of_demonstrations

if __name__ == '__main__':
    import pprint
    pp = pprint.PrettyPrinter(indent=4)

    base_path = os.path.dirname(os.path.realpath(__file__))
    er = ExperimentRecord('./test_data/experiment_at_2018y05m09d21H36M14S')


    print '\ntag_ranges', '-'*20
    pp.pprint(er.tag_ranges)
    print '\nunsuccessful_tag_ranges', '-'*20
    pp.pprint(er.unsuccessful_tag_ranges)
    print '\nsuccessful_tag_ranges', '-'*20
    pp.pprint(er.successful_tag_ranges)
    print '\nanomaly_signals', '-'*20
    pp.pprint(er.anomaly_signals)
    print '\nlist_of_anomalies', '-'*20
    pp.pprint(er.list_of_anomalies)
    print '\nlist_of_demonstrations', '-'*20
    pp.pprint(er.list_of_demonstrations)
