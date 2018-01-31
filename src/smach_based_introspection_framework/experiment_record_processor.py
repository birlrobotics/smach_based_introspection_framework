from birl_offline_data_handler.rosbag_handler import (
    RosbagHandler,
    InvalidRosbagPath,
    TopicNotFoundInRosbag,
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
                
        
             

    
