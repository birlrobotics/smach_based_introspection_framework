import os
import sys
import glob
from birl_offline_data_handler.rosbag_handler import (
    RosbagHandler,
    InvalidRosbagPath,
    TopicNotFoundInRosbag,
)
from birl_offline_data_handler.rosbag_anomaly_extractor import (
    RosbagAnomalyExtractor
)
from _constant import (
    SUCCESSULLY_EXECUTED_SKILL,
    UNSUCCESSFULLY_EXECUTED_SKILL,
    ROLLBACK_RECOVERY_TAG,
    RECOVERY_DEMONSTRATED_BY_HUMAN_TAG,
    folder_time_fmt,
    RECOVERY_SKILL_BEGINS_AT,
)
import pprint
pp = pprint.PrettyPrinter(indent=4)
from smach_based_introspection_framework._constant import (
    experiment_record_folder, 
    dataset_folder,
    skill_sensory_data_folder_folder,
    anomaly_label_file,
)
import shutil
import datetime
import json

def get_anomaly_labels(exp_dir):
    txt_path = os.path.join(exp_dir, anomaly_label_file)
    lines = open(txt_path, 'r').readlines()
    labels = [i.strip() for i in lines]
    return [i for i in labels if i != ""]

def get_tag_range(df):
    ret = [] 
    last_tag = None
    last_tag_at = 0
    for index, tag in df['.tag'].iteritems():
        if tag != last_tag:
            if last_tag is not None:
                new_range = (last_tag, (last_tag_at, index-1))
                ret.append(new_range)     
            last_tag = tag
            last_tag_at = index
    new_range = (last_tag, (last_tag_at, len(df['.tag'])-1))
    ret.append(new_range)     

    return ret

def get_latest_dataset_dir():
    if hasattr(get_latest_dataset_dir, "latest_dataset_dir"):
        return get_latest_dataset_dir.latest_dataset_dir
    latest_dataset_dir = os.path.join(dataset_folder, 'latest')
    if os.path.isdir(latest_dataset_dir):
       shutil.move(
            latest_dataset_dir,  
            os.path.join(dataset_folder, "data_set_dir.old.%s"%datetime.datetime.now().strftime(folder_time_fmt))
        )
        
    os.makedirs(latest_dataset_dir)
    get_latest_dataset_dir.latest_dataset_dir = latest_dataset_dir
    return latest_dataset_dir
    
def get_recovery_skill_tag(nominal_skill_tag, anomaly_type):
    if anomaly_type == "count":
        raise Exception("anomaly type CANNOT be \"count\"")
    if hasattr(get_recovery_skill_tag, "lookup_dict"):
        lookup_dict = get_recovery_skill_tag.lookup_dict
    else:
        p = os.path.join(get_latest_dataset_dir(), "recovery_tag_lookup_dict.json")
        try:
            lookup_dict = json.load(p)
        except:
            lookup_dict = {'count':RECOVERY_SKILL_BEGINS_AT}
        get_recovery_skill_tag.lookup_dict = lookup_dict

    key = "nominal_skill_%s_anomaly_type_%s"%(nominal_skill_tag, anomaly_type)
    if key in lookup_dict:
        return lookup_dict[key]
    
    lookup_dict[key] = lookup_dict["count"] 
    lookup_dict["count"] += 1
    json.dump(lookup_dict, open(os.path.join(get_latest_dataset_dir(), "recovery_tag_lookup_dict.json"),'w'))
    return lookup_dict[key]

def add_skill_introspection_data(tag, df, name):
    tag_dir = os.path.join(
        get_latest_dataset_dir(),
        'skill_data',
        "tag_%s"%tag,
    )
    if not os.path.isdir(tag_dir):
        os.makedirs(tag_dir)
    df.to_csv(os.path.join(tag_dir, name), sep=',')

def add_anomaly_data(nominal_skill_tag, anomaly_type, df, name):
    key = "%s_%s"%(nominal_skill_tag, anomaly_type)
    anomaly_dir = os.path.join(
        get_latest_dataset_dir(),
        'anomaly_data',
        "nominal_skill_%s_anomaly_type_%s"%(nominal_skill_tag, anomaly_type)
    )
    if not os.path.isdir(anomaly_dir):
        os.makedirs(anomaly_dir)
    df.to_csv(os.path.join(anomaly_dir, name), sep=',')
    pass
                
def run():        
    if not os.path.isdir(experiment_record_folder):
        print experiment_record_folder, "not found."
        sys.exit(0)
    
    exp_dirs = [i for i in glob.glob(os.path.join(experiment_record_folder, '*')) if os.path.isdir(i)]

    for exp_dir in exp_dirs: 
        print exp_dir
        try:
            rh = RosbagHandler(os.path.join(exp_dir, "record.bag"))
        except InvalidRosbagPath as e:
            raise Exception("Can't find record.bag in %s"%exp_dir) 
        ret = rh.get_csv_of_a_topic("/tag_multimodal") 
        if len(ret) != 1:
            raise Exception("Failed to get /tag_multimodal from record.bag in %s"%exp_dir)
        bag_path, tag_df = ret[0]
        list_of_tag_range = [i for i in get_tag_range(tag_df) if i[0] != 0]

        stat = {
            SUCCESSULLY_EXECUTED_SKILL: [],
            UNSUCCESSFULLY_EXECUTED_SKILL: [],
        }
       
        last_nominal_skill_idx = None 
        for idx, tr_tuple in enumerate(list_of_tag_range):
            tag, ran = tr_tuple
            if idx == len(list_of_tag_range)-1:
                next_tag = None
            else:
                next_tag = list_of_tag_range[idx+1][0]

            # Nominal skill
            if tag > 0:
                if tag < RECOVERY_SKILL_BEGINS_AT:
                    last_nominal_skill_idx = idx
                if next_tag is None or next_tag > 0:
                    stat[SUCCESSULLY_EXECUTED_SKILL].append(tr_tuple)  
                else:
                    stat[UNSUCCESSFULLY_EXECUTED_SKILL].append({
                        "failed_nominal_skill": list_of_tag_range[last_nominal_skill_idx],
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
            continue

        rae = RosbagAnomalyExtractor(os.path.join(exp_dir, "record.bag"))
        ret = rae.get_anomaly_csv(
            "/tag_multimodal",
            "/anomaly_detection_signal",
            4,
            10,
        )
        if len(ret) != 1:
            raise Exception("Failed to extract anomalies from record.bag in %s"%exp_dir)
        bag_path, list_of_anomaly = ret[0]

        if len(list_of_anomaly) != len(stat[UNSUCCESSFULLY_EXECUTED_SKILL]):
            raise Exception("Anomalies amount does NOT match anomaly amount.")

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


            df = i['extracted_anomaly'][1]
            add_anomaly_data(nominal_skill_tag, anomaly_type, df, "no_%s_anomaly_in_%s"%(count, os.path.basename(exp_dir)))
