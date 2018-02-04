import process_experiment_record_to_dataset as e2d
import glob
import os
from _constant import (
    folder_time_fmt,
    dataset_folder,
    anomaly_label_file,
    model_folder,
)
import re
import pandas as pd
from sklearn.model_selection import train_test_split
from birl_hmm.hmm_training import train_model, hmm_util
from sklearn.externals import joblib
import shutil
import datetime
import json

data_fields_store = {
    "endpoint_pose": [
        '.endpoint_state.pose.position.x',
        '.endpoint_state.pose.position.y',
        '.endpoint_state.pose.position.z',
        '.endpoint_state.pose.orientation.x',
        '.endpoint_state.pose.orientation.y',
        '.endpoint_state.pose.orientation.z',
        '.endpoint_state.pose.orientation.w'
    ],
    'wrench': [
         '.wrench_stamped.wrench.force.x',
         '.wrench_stamped.wrench.force.y',
         '.wrench_stamped.wrench.force.z',
         '.wrench_stamped.wrench.torque.x',
         '.wrench_stamped.wrench.torque.y',
         '.wrench_stamped.wrench.torque.z',
    ] 
}
data_type_chosen = 'endpoint_pose_and_wrench'        
data_type_split = data_type_chosen.split("_and_")
interested_data_fields = []
for data_type in data_type_split:
    interested_data_fields += data_fields_store[data_type]
model_type = 'hmmlearn\'s HMM'
score_metric = '_score_metric_sum_of_loglik_'
model_config = {
    'hmm_max_train_iteration': 1000,
    'hmm_max_hidden_state_amount': 7,
    'gaussianhmm_covariance_type_string': ['diag', 'spherical', 'full', 'tied'],
}


def get_latest_model_dir():
    if hasattr(get_latest_model_dir, "latest_model_dir"):
        return get_latest_model_dir.latest_model_dir
    latest_model_dir = os.path.join(model_folder, 'latest')
    if os.path.isdir(latest_model_dir):
       shutil.move(
            latest_model_dir,  
            os.path.join(model_folder, "model_folder.old.%s"%datetime.datetime.now().strftime(folder_time_fmt))
        )
        
    os.makedirs(latest_model_dir)
    get_latest_model_dir.latest_model_dir = latest_model_dir
    return latest_model_dir

def run():
    latest_dataset_dir = os.path.join(dataset_folder, 'latest')
    if not os.path.isdir(latest_dataset_dir):
        raise Exception("Not found %s"%latest_dataset_dir)

    prog = re.compile(r'tag_(\d+)')
    for i in glob.glob(os.path.join(latest_dataset_dir, "skill_data", '*')):
        tag = prog.match(os.path.basename(i)).group(1)

        list_of_mat = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_mat.append(df[interested_data_fields].values)
    
        list_of_train_mat, list_of_test_mat = train_test_split(list_of_mat, test_size=0.25)
        
        if len(list_of_train_mat) == 0:
            print 'Skill with tag %s contains 0 train samples, failed to train introspection model for it'%tag
            continue
        else:
            print 'Skill with tag %s contains %s train samples'%(tag, len(list_of_train_mat))
            

        if len(list_of_test_mat) == 0:
            print 'Skill with tag %s contains 0 test samples, failed to train introspection model for it'%tag
            continue
        else:
            print 'Skill with tag %s contains %s test samples'%(tag, len(list_of_test_mat))
    

        sorted_model_list = train_model.run(
            list_of_train_mat=list_of_train_mat, 
            list_of_test_mat=list_of_test_mat,
            model_type=model_type,
            model_config=model_config,
            score_metric=score_metric,
        )
        best = sorted_model_list[0]
            
        d = os.path.join(get_latest_model_dir(), 'tag_%s'%tag)
        if not os.path.isdir(d):
            os.makedirs(d)
        joblib.dump(
            best['model'],
            os.path.join(d, 'introspection_model')
        )
    
        train_report = [{hmm_util.get_model_config_id(i['now_model_config']): i['score']} for i in sorted_model_list]
        json.dump(
            [ 
                {'data_type_chosen': data_type_chosen},
                {'model_type': model_type},
                {'model_config': model_config},
                {'score_metric': score_metric},
                {'best': best['now_model_config']},
                {'train_report': train_report},
            ],
            open(os.path.join(d, 'model_info'), 'w'),
            indent=4,
        )
