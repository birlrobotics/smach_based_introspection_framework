from sklearn.model_selection import train_test_split
from birl_dmp.dmp_training import train_model
import numpy as np
import ipdb

default_model_type = 'pydmps'
default_model_config = {
    'gen_ay': range(5, 50, 5),
}

def run(mat, model_type=default_model_type, model_config=default_model_config):

    sorted_model_list = train_model.run(
        mat,
        model_type,
        model_config,
    )
    best = sorted_model_list[0]
    train_report = [{str(i['now_model_config']): i['score']} for i in sorted_model_list]

    return {
        'model': best['model'],
        'model_info': {
            'config_of_best_model': best['now_model_config'],
            'train_report': train_report,
        }
    }
