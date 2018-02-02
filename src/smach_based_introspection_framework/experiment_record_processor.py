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
    from smach_based_introspection_framework._constant import introspection_data_folder
    if not os.path.isdir(introspection_data_folder):
        print introspection_data_folder, "not found."
        sys.exit(0)
    
    exp_dirs = [i for i in glob.glob(os.path.join(introspection_data_folder, '*')) if os.path.isdir(i)]

    for exp_dir in exp_dirs: 
        print exp_dir
        try:
            rh = RosbagHandler(os.path.join(exp_dir, "record.bag"))
        except InvalidRosbagPath as e:
            raise Exception("Can't find record.bag in %s"%exp_dir) 
        
        l = rh.get_csv_of_a_topic("/tag_multimodal") 
        if len(l) != 1:
            raise Exception("Failed to get /tag_multimodal from record.bag in %s"%exp_dir)
        bag_path, tag_df = l[0]
        list_of_tag_range = get_tag_range(tag_df)
        print list_of_tag_range


        rae = RosbagAnomalyExtractor(os.path.join(exp_dir, "record.bag"))
        ret = rae.get_anomaly_csv(
            "/tag_multimodal",
            "/anomaly_detection_signal",
            4,
            10,
        )
        for i in ret:
            print i[0]
            for j in i[1]:
                print j[0]
             

    
