import os
import sys
import pickle
import glob
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
from smach_based_introspection_framework.configurables import (
    anomaly_window_size_in_sec, 
    anomaly_classification_timeseries_hz, 
)
import ipdb
import coloredlogs, logging
coloredlogs.install()

def get_anomaly_labels(exp_dir):
    txt_path = os.path.join(exp_dir, anomaly_label_file)
    lines = open(txt_path, 'r').readlines()
    labels = [i.strip() for i in lines]
    return [i for i in labels if i != ""]

def get_tag_range(df):
    ret = [] 
    last_tag = None
    last_tag_at = 0
    for index, tu in enumerate(df['.tag'].iteritems()):
        tag = tu[1]
        if tag != last_tag:
            if last_tag is not None:
                new_range = (last_tag, (last_tag_at, index-1))
                ret.append(new_range)     
            last_tag = tag
            last_tag_at = index
    new_range = (last_tag, (last_tag_at, len(df['.tag'])-1))
    ret.append(new_range)     

    return ret

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

def temporary_solution_to_add_wrench_derivative(df):
    if u'.wrench_stamped.wrench.force.x' not in df.columns:
        return df
    df['.delta_wrench.force.x'] = df['.wrench_stamped.wrench.force.x']
    df['.delta_wrench.force.y'] = df['.wrench_stamped.wrench.force.y']
    df['.delta_wrench.force.z'] = df['.wrench_stamped.wrench.force.z']

    df['.delta_wrench.force.x'].iloc[1:] = df['.delta_wrench.force.x'].iloc[1:].values-df['.delta_wrench.force.x'].iloc[:-1].values
    df['.delta_wrench.force.x'].iloc[0] = 0
    df['.delta_wrench.force.y'].iloc[1:] = df['.delta_wrench.force.y'].iloc[1:].values-df['.delta_wrench.force.y'].iloc[:-1].values
    df['.delta_wrench.force.y'].iloc[0] = 0
    df['.delta_wrench.force.z'].iloc[1:] = df['.delta_wrench.force.z'].iloc[1:].values-df['.delta_wrench.force.z'].iloc[:-1].values
    df['.delta_wrench.force.z'].iloc[0] = 0

    df['.delta_wrench.torque.x'] = df['.wrench_stamped.wrench.torque.x']
    df['.delta_wrench.torque.y'] = df['.wrench_stamped.wrench.torque.y']
    df['.delta_wrench.torque.z'] = df['.wrench_stamped.wrench.torque.z']

    df['.delta_wrench.torque.x'].iloc[1:] = df['.delta_wrench.torque.x'].iloc[1:].values-df['.delta_wrench.torque.x'].iloc[:-1].values
    df['.delta_wrench.torque.x'].iloc[0] = 0
    df['.delta_wrench.torque.y'].iloc[1:] = df['.delta_wrench.torque.y'].iloc[1:].values-df['.delta_wrench.torque.y'].iloc[:-1].values
    df['.delta_wrench.torque.y'].iloc[0] = 0
    df['.delta_wrench.torque.z'].iloc[1:] = df['.delta_wrench.torque.z'].iloc[1:].values-df['.delta_wrench.torque.z'].iloc[:-1].values
    df['.delta_wrench.torque.z'].iloc[0] = 0

    return df

def add_skill_introspection_data(tag, df, name):
    tag_dir = os.path.join(
        latest_dataset_folder,
        'skill_data',
        "tag_%s"%tag,
    )
    if not os.path.isdir(tag_dir):
        os.makedirs(tag_dir)
    df = temporary_solution_to_add_wrench_derivative(df)
    df.to_csv(os.path.join(tag_dir, name), sep=',')

def add_anomaly_data(nominal_skill_tag, anomaly_type, df, name):
    key = "%s_%s"%(nominal_skill_tag, anomaly_type)
    anomaly_dir = os.path.join(
        latest_dataset_folder,
        'anomaly_data',
        "nominal_skill_%s_anomaly_type_%s"%(nominal_skill_tag, anomaly_type)
    )
    if not os.path.isdir(anomaly_dir):
        os.makedirs(anomaly_dir)
    df = temporary_solution_to_add_wrench_derivative(df)
    df.to_csv(os.path.join(anomaly_dir, name), sep=',')
    pass

def process_time_and_set_as_index(df):
    from dateutil import parser
    import numpy as np
    import datetime
    df['time'] = df['time'].apply(lambda x: parser.parse(x))
    start_datetime = df['time'][0]
    df['time'] -= datetime.datetime(1970, 1, 1, 0, 0, 0, 0)
    df['time'] = df['time'].apply(lambda x: x.total_seconds())
    df = df.drop_duplicates(subset='time').set_index('time')
    return df

def get_log_dict():
    log_file_path = os.path.join(latest_dataset_folder, "log_dict.pkl")
    if os.path.isfile(log_file_path):
        log_dict = pickle.load(open(log_file_path, 'r'))
    else:
        log_dict = {}
    return log_dict

def save_log_dict(log_dict):
    log_file_path = os.path.join(latest_dataset_folder, "log_dict.pkl")
    pickle.dump(log_dict, open(log_file_path, 'w'))
        
                
def run():        
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.INFO)
    logger.addHandler(consoleHandler)


    if not os.path.isdir(experiment_record_folder):
        print experiment_record_folder, "not found."
        sys.exit(0)

    setup_latest_dataset_folder()
    
    exp_dirs = [i for i in glob.glob(os.path.join(experiment_record_folder, '*')) if os.path.isdir(i)]

    # SORTING IS CRITICAL, 
    # we have to process experiments in temporal order
    # otherwise recovery tag assignments will crash
    exp_dirs.sort() 

    log_dict = get_log_dict()
    for exp_dir in exp_dirs: 
        if exp_dir in log_dict:
            continue
        try:
            print exp_dir
            try:
                rh = RosbagHandler(os.path.join(exp_dir, "record.bag"))
            except InvalidRosbagPath as e:
                raise Exception("Can't find record.bag in %s"%exp_dir) 
            ret = rh.get_csv_of_a_topic("/tag_multimodal") 
            if len(ret) != 1:
                raise Exception("Failed to get /tag_multimodal from record.bag in %s"%exp_dir)
            bag_path, tag_df = ret[0]
            tag_df = process_time_and_set_as_index(tag_df)

            try:
                ret = rh.get_csv_of_a_topic("/observation/goal_vector") 
                if len(ret) != 1:
                    raise Exception("Failed to get /observation/goal_vector from record.bag in %s"%exp_dir)
                bag_path, goal_df = ret[0]
                goal_df = process_time_and_set_as_index(goal_df)

                goal_df = goal_df.reindex(goal_df.index.union(tag_df.index), method='nearest')
                goal_df = goal_df.loc[tag_df.index]
                tag_df['.goal_vector'] =  goal_df['.goal_vector']
            except TopicNotFoundInRosbag:
                pass

            list_of_tag_range = [i for i in get_tag_range(tag_df) if i[0] != 0]

            stat = {
                SUCCESSULLY_EXECUTED_SKILL: [],
                UNSUCCESSFULLY_EXECUTED_SKILL: [],
            }
           
            for idx, tr_tuple in enumerate(list_of_tag_range):
                tag, ran = tr_tuple
                if idx == len(list_of_tag_range)-1:
                    next_tag = None
                else:
                    next_tag = list_of_tag_range[idx+1][0]

                # Nominal skill
                if tag > 0:
                    if next_tag is None or next_tag > 0:
                        stat[SUCCESSULLY_EXECUTED_SKILL].append(tr_tuple)  
                    else:
                        stat[UNSUCCESSFULLY_EXECUTED_SKILL].append({
                            "failed_nominal_skill": list_of_tag_range[idx],
                            "human_dem_recovery_tr_tuple": None,
                            "extracted_anomaly": None,
                            "anomaly_label": None
                        })
                elif tag == RECOVERY_DEMONSTRATED_BY_HUMAN_TAG:
                    stat[UNSUCCESSFULLY_EXECUTED_SKILL][-1]['human_dem_recovery_tr_tuple'] = tr_tuple


            for count, i in enumerate(stat[SUCCESSULLY_EXECUTED_SKILL]):
                tag = i[0]
                ran = i[1]
                df = tag_df.iloc[ran[0]:ran[1]]
                add_skill_introspection_data(i[0], df, "no_%s_successful_skill_from_%s"%(count, os.path.basename(exp_dir)))

            if len(stat[UNSUCCESSFULLY_EXECUTED_SKILL]) == 0:
                pass
            else:
                rae = RosbagAnomalyExtractor(os.path.join(exp_dir, "record.bag"))
                ret = rae.get_anomaly_csv(
                    "/tag_multimodal",
                    "/anomaly_detection_signal",
                    anomaly_window_size_in_sec,
                    anomaly_classification_timeseries_hz,
                )
                if len(ret) != 1:
                    raise Exception("Failed to extract anomalies from record.bag in %s"%exp_dir)
                bag_path, list_of_anomaly = ret[0]

                if len(list_of_anomaly) != len(stat[UNSUCCESSFULLY_EXECUTED_SKILL]):
                    raise Exception("Anomalies amount does NOT match UNSUCCESSFULLY_EXECUTED_SKILL amount.")

                list_of_anomaly_label = get_anomaly_labels(exp_dir)
                if len(list_of_anomaly) != len(list_of_anomaly_label):
                    raise Exception("Anomalies amount does NOT match anomaly label amount.")
                
            
                for idx, anomaly_tuple in enumerate(list_of_anomaly):
                    stat[UNSUCCESSFULLY_EXECUTED_SKILL][idx]["extracted_anomaly"] = anomaly_tuple
                    stat[UNSUCCESSFULLY_EXECUTED_SKILL][idx]["anomaly_label"] = list_of_anomaly_label[idx]
                       
                for count, i in enumerate(stat[UNSUCCESSFULLY_EXECUTED_SKILL]):
                    nominal_skill_tag = i['failed_nominal_skill'][0]
                    anomaly_type = i['anomaly_label']
                    recovery_demonstration = i['human_dem_recovery_tr_tuple']
                    if recovery_demonstration is not None:
                        tag = get_recovery_skill_tag(nominal_skill_tag, anomaly_type) 
                        ran = recovery_demonstration[1]
                        df = tag_df.iloc[ran[0]:ran[1]]
                        add_skill_introspection_data(tag, df, "recovery_demonstration_for_no_%s_anomaly_in_%s"%(count, os.path.basename(exp_dir)))
                        # TODO save old and new goals

                    df = i['extracted_anomaly'][1]
                    add_anomaly_data(nominal_skill_tag, anomaly_type, df, "no_%s_anomaly_in_%s"%(count, os.path.basename(exp_dir)))
        except Exception as e:
            logger.error("process exp_dir \"%s\"failed: %s"%(exp_dir,e ))
        else:
            log_dict[exp_dir] = True
        save_log_dict(log_dict)
