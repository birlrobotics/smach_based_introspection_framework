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
)
import pprint
pp = pprint.PrettyPrinter(indent=4)
from smach_based_introspection_framework._constant import experiment_record_folder, skill_sensory_data_folder_folder

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
        print list_of_tag_range

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
                        "incomplete_skill": tr_tuple,
                        "human_dem_recovery_tr_tuple": None,
                        "extracted_anomaly": None,
                    })
            elif tag == RECOVERY_DEMONSTRATED_BY_HUMAN_TAG:
                stat[UNSUCCESSFULLY_EXECUTED_SKILL][-1]['recovery_tr_tuple'] = tr_tuple
                        
        if len(stat[UNSUCCESSFULLY_EXECUTED_SKILL]) == 0:
            pp.pprint(stat)
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
            raise Exception("Anomalies amount does NOT match incomplete skill amount.")
    
        for idx, anomaly_tuple in enumerate(list_of_anomaly):
            stat[UNSUCCESSFULLY_EXECUTED_SKILL][idx]["extracted_anomaly"] = anomaly_tuple
               

    
