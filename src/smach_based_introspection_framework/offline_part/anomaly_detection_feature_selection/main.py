from smach_based_introspection_framework import ExperimentRecord
import glob
from smach_based_introspection_framework._constant import (
    experiment_record_folder, 
)
import os
from rostopics_to_timeseries import OfflineRostopicsToTimeseries
from filter_cfgs import filter_cfgs
import itertools
import ipdb


if __name__ == '__main__':
    
    exp_dirs = [i for i in glob.glob(os.path.join(experiment_record_folder, '*')) if os.path.isdir(i)]
    exp_dirs.sort() 
    for filter_cfg, ed in itertools.product(filter_cfgs, exp_dirs):
        er = ExperimentRecord(ed)
        ortt = OfflineRostopicsToTimeseries(filter_cfg, rate=10) 
        for tag, (st, et) in er.successful_tag_ranges:
            t, mat = ortt.get_timeseries_mat(
                er.rosbag,
                st,
                et,
            )
            print mat
            ipdb.set_trace()
