import os
import sys
import rospy
import pickle
import glob
import traceback

from birl_offline_data_handler.rosbag_handler import (
    RosbagHandler,
    InvalidRosbagPath,
    TopicNotFoundInRosbag,
)
from birl_offline_data_handler.rosbag_anomaly_extractor import (
    RosbagAnomalyExtractor
)
from smach_based_introspection_framework._constant import (
    SUCCESSULLY_EXECUTED_SKILL,
    UNSUCCESSFULLY_EXECUTED_SKILL,
    ROLLBACK_RECOVERY_TAG,
    RECOVERY_DEMONSTRATED_BY_HUMAN_TAG,
    folder_time_fmt,
    RECOVERY_SKILL_BEGINS_AT,
    experiment_record_folder, 
    dataset_folder,
    latest_dataset_folder,
    anomaly_label_file,
)
import pprint
pp = pprint.PrettyPrinter(indent=4)
import shutil
import datetime
import json
import ipdb
import logging
import sys
from smach_based_introspection_framework import ExperimentRecord
from rostopics_to_timeseries import OfflineRostopicsToTimeseries
import pandas as pd

def generate_and_save_csv(output_csv, er, st, et, filtering_scheme, ortt, logger):
    if not os.path.isfile(output_csv):
        try:
            t, mat = ortt.get_timeseries_mat(
                er.rosbag,
                st,
                et,
            )
        except Exception as e:
            traceback.print_exc()
            logger.error("Fail to get timeseries_mat: %s"%e)
            raise e
        
        df = pd.DataFrame(mat, columns=filtering_scheme.timeseries_header, index=t)
        output_dir = os.path.dirname(output_csv)
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        df.to_csv(output_csv)
        logger.debug("Done.")
    else:
        logger.debug("Already done, gonna skip.")

def setup_latest_dataset_folder():
    if not os.path.isdir(latest_dataset_folder):
        os.makedirs(latest_dataset_folder)
    
def get_recovery_skill_tag(nominal_skill_tag, anomaly_type, add_if_not_exist=True):
    if anomaly_type == "count":
        raise Exception("anomaly type CANNOT be \"count\"")
    if hasattr(get_recovery_skill_tag, "lookup_dict"):
        lookup_dict = get_recovery_skill_tag.lookup_dict
    else:
        p = os.path.join(latest_dataset_folder, "recovery_tag_lookup_dict.json")
        try:
            lookup_dict = json.load(open(p, 'r'))
        except:
            lookup_dict = {'count':RECOVERY_SKILL_BEGINS_AT}
        get_recovery_skill_tag.lookup_dict = lookup_dict

    key = "nominal_skill_%s_anomaly_type_%s"%(nominal_skill_tag, anomaly_type)
    print key, lookup_dict
    if key in lookup_dict:
        return lookup_dict[key]
    elif not add_if_not_exist:
        return None
    
    lookup_dict[key] = lookup_dict["count"] 
    lookup_dict["count"] += 1
    json.dump(lookup_dict, open(os.path.join(latest_dataset_folder, "recovery_tag_lookup_dict.json"),'w'))
    return lookup_dict[key]
                
def run():        
    logger = logging.getLogger()

    if not os.path.isdir(experiment_record_folder):
        print experiment_record_folder, "not found."
        sys.exit(0)

    setup_latest_dataset_folder()
    
    exp_dirs = [i for i in glob.glob(os.path.join(experiment_record_folder, '*')) if os.path.isdir(i)]

    # SORTING IS CRITICAL, 
    # we have to process experiments in temporal order
    # otherwise recovery tag assignments will crash
    exp_dirs.sort() 

    for exp_dir in exp_dirs: 
        try:
            logger.debug("Processing "+exp_dir)

            er = ExperimentRecord(exp_dir)

            from smach_based_introspection_framework.configurables import anomaly_detection_timeseries_config
            ortt = OfflineRostopicsToTimeseries(anomaly_detection_timeseries_config) 
            for count, (tag, (st, et)) in enumerate(er.successful_tag_ranges):
                logger.debug("No.%s successful skill"%count)
                output_csv = os.path.join(
                    latest_dataset_folder,
                    'skill_data',
                    "tag_%s"%tag,
                    "no_%s_successful_skill_from_%s"%(count, os.path.basename(exp_dir)),
                )
                generate_and_save_csv(output_csv, er, st, et, anomaly_detection_timeseries_config, ortt, logger)


            from smach_based_introspection_framework.configurables import anomaly_window_size, anomaly_classification_timeseries_config
            secs_before, secs_after = anomaly_window_size
            ortt = OfflineRostopicsToTimeseries(anomaly_classification_timeseries_config) 

            for count, anomaly in enumerate(er.list_of_anomalies):
                logger.debug("anomaly_type: %s, anomaly_time: %s"%(anomaly.label, anomaly.time)) 
                output_csv = os.path.join(
                    latest_dataset_folder,
                    'anomaly_data',
                    "anomaly_type_%s"%(anomaly.label,),
                    "no_%s_anomaly_in_%s"%(count, os.path.basename(exp_dir)),
                )
                generate_and_save_csv(output_csv, er, anomaly.time-rospy.Duration(secs_before), anomaly.time+rospy.Duration(secs_after), anomaly_classification_timeseries_config, ortt, logger)


            from smach_based_introspection_framework.configurables import dmp_cmd_timeseries_config
            ortt = OfflineRostopicsToTimeseries(dmp_cmd_timeseries_config) 
            for count, demonstration in enumerate(er.list_of_demonstrations):
                logger.debug("No.%s demonstration: %s"%(count, demonstration))
                demonstration.tag = str(get_recovery_skill_tag(demonstration.targeted_anomaly.skill_belonged_to.tag, demonstration.targeted_anomaly.label))

                output_csv = os.path.join(
                    latest_dataset_folder,
                    'demonstration_data',
                    "nominal_skill_%s_anomaly_type_%s_tag_%s"%(demonstration.targeted_anomaly.skill_belonged_to.tag, demonstration.targeted_anomaly.label, demonstration.tag),
                    "No.%s_demonstration_in_%s"%(count, os.path.basename(exp_dir)),
                    "dmp_cmd_timeseries.csv",
                )
                generate_and_save_csv(output_csv, er, demonstration.start_time, demonstration.end_time, dmp_cmd_timeseries_config, ortt, logger)

                output_json = os.path.join(
                    os.path.dirname(output_csv),
                    "original_goal.json",
                )
                with open(output_json, 'w') as f:
                    json.dump(demonstration.original_goal, f)
        except Exception as e:
            logger.error("process exp_dir \"%s\"failed: %s"%(exp_dir,e ))

if __name__=="__main__":
    run()
