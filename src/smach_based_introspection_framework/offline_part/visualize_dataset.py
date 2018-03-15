import process_experiment_record_to_dataset as e2d
import glob
import os
from smach_based_introspection_framework._constant import (
    folder_time_fmt,
    latest_dataset_folder,
    latest_visualized_dataset_folder,
    anomaly_label_file,
    visualized_dataset_folder,
    RECOVERY_SKILL_BEGINS_AT
)
import re
import pandas as pd
from sklearn.externals import joblib
import shutil
import datetime
import json
import numpy as np
import ipdb
import smach_based_introspection_framework.configurables as configurables 
import logging
import matplotlib.pyplot as plt

dmp_cmd_fields  = configurables.dmp_cmd_fields  
data_type_chosen  = configurables.data_type_chosen  
interested_data_fields  = configurables.interested_data_fields  
model_type  = configurables.model_type  
score_metric  = configurables.score_metric  
model_config  = configurables.model_config  

def plot_list_of_df(save_folder, list_of_df, list_of_labels):
    print 'plot_list_of_df with save_folder %s'%save_folder
    if len(list_of_df) == 0:
        return
    dims = list_of_df[0].columns
    for dim in dims:
        fig, ax = plt.subplots(nrows=1, ncols=1)
        lgds = []

        for idx, df in enumerate(list_of_df):
            label = list_of_labels[idx]
            ax.plot(df[dim], label=label)

        ax.set_title('...'+(save_folder+', '+dim)[-70:])

        handles, labels = ax.get_legend_handles_labels()
        lgd = ax.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5,-0.1))

        fig.savefig(os.path.join(save_folder, dim.replace('.', '>')), format='png', bbox_extra_artists=(lgd,), bbox_inches='tight')

        plt.close(fig)
    pass

def get_latest_visualized_dataset_folder():
    if hasattr(get_latest_visualized_dataset_folder, "latest_visualized_dataset_folder"):
        return get_latest_visualized_dataset_folder.latest_visualized_dataset_folder
    if os.path.isdir(latest_visualized_dataset_folder):
       shutil.move(
            latest_visualized_dataset_folder,  
            os.path.join(visualized_dataset_folder, "visualized_dataset_folder.old.%s"%datetime.datetime.now().strftime(folder_time_fmt))
        )
        
    os.makedirs(latest_visualized_dataset_folder)
    get_latest_visualized_dataset_folder.latest_visualized_dataset_folder = latest_visualized_dataset_folder
    return latest_visualized_dataset_folder

def run():
    if not os.path.isdir(latest_dataset_folder):
        raise Exception("Not found %s"%latest_dataset_folder)

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger('process_dataset_to_models')

    log_file = os.path.join(get_latest_visualized_dataset_folder(), 'run.log')
    fileHandler = logging.FileHandler(os.path.realpath(log_file))
    logger.addHandler(fileHandler)

    consoleHandler = logging.StreamHandler()
    logger.addHandler(consoleHandler)

    prog = re.compile(r'tag_(\d+)')
    for i in glob.glob(os.path.join(latest_dataset_folder, "skill_data", '*')):
        tag = prog.match(os.path.basename(i)).group(1)

        list_of_df = []
        list_of_labels = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_df.append(df[interested_data_fields])
            list_of_labels.append(os.path.basename(j))

        d = os.path.join(get_latest_visualized_dataset_folder(), 'tag_%s'%tag)
        if not os.path.isdir(d):
            os.makedirs(d)

        plot_list_of_df(d, list_of_df, list_of_labels)

    prog = re.compile(r'nominal_skill_(\d+)_anomaly_type_(.*)')
    for i in glob.glob(os.path.join(latest_dataset_folder, "anomaly_data", '*')):
        m = prog.match(os.path.basename(i))
        tag = m.group(1)
        anomaly_type = m.group(2)
        if anomaly_type == 'Unlabeled':
            continue
        list_of_df = []
        list_of_labels = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_df.append(df[interested_data_fields])
            list_of_labels.append(os.path.basename(j))

        d = os.path.join(get_latest_visualized_dataset_folder(), os.path.basename(i))
        if not os.path.isdir(d):
            os.makedirs(d)

        plot_list_of_df(d, list_of_df, list_of_labels)

    prog = re.compile(r'tag_(\d+)')
    for i in glob.glob(os.path.join(latest_dataset_folder, "skill_data", '*')):
        tag = prog.match(os.path.basename(i)).group(1)
        if int(tag) < RECOVERY_SKILL_BEGINS_AT:
            continue
        list_of_df = []
        list_of_labels = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_df.append(df[dmp_cmd_fields])
            list_of_labels.append(os.path.basename(j))

        d = os.path.join(get_latest_visualized_dataset_folder(), os.path.basename(i))
        if not os.path.isdir(d):
            os.makedirs(d)

        plot_list_of_df(d, list_of_df, list_of_labels)


    fileHandler.close()


if __name__ == '__main__':
    run()







