
from smach_based_introspection_framework._constant import (
    datasets_of_filtering_schemes_folder,
)
import glob
import os
import coloredlogs, logging
from smach_based_introspection_framework.online_part.anomaly_detector import Detectors 
from sklearn.externals import joblib
import pandas as pd
import numpy as np
import pickle
import ipdb

def get_first_anomaly_signal_time(model, timeseries_mat, ts):
    detector = Detectors.DetectorBasedOnGradientOfLoglikCurve(
        {1: model['hmm_model']}, 
        {1: model['threshold_for_introspection']},
    )
    for idx, t in enumerate(ts):
        now_skill, anomaly_detected, metric, threshold = detector.add_one_smaple_and_identify_skill_and_detect_anomaly(np.array(timeseries_mat[idx]).reshape(1,-1), now_skill=1)
        if anomaly_detected:
            return t
    return None

def run():
    logger = logging.getLogger('CollDetectStat')
    logger.setLevel(logging.INFO)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.INFO)
    logger.addHandler(consoleHandler)

    succ_folders = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'No.* filtering scheme',
        'successful_skills',
        'skill *',
    )) 

    unsucc_folders = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'No.* filtering scheme',
        'unsuccessful_skills',
        'skill *',
    ))

    model_folders = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'introspection_models',
        'No.* filtering scheme', 
        'successful_skills',
        'skill *',
    ))

    for model_folder in model_folders:
        logger.info(os.path.realpath(model_folder))
        model = joblib.load(os.path.join(model_folder, 'introspection_model'))
        path_postfix = os.path.relpath(model_folder, os.path.join(datasets_of_filtering_schemes_folder, 'introspection_models'))
        succ_folder = os.path.join(datasets_of_filtering_schemes_folder, path_postfix)
        unsucc_folder = succ_folder.replace("successful_skills", "unsuccessful_skills")
        model_stat = {
            "TP": 0,
            "TN": 0,
            "FP": 0,
            "FN": 0,
        }
        stat_df = pd.DataFrame(columns=['sample name', 'anomaly type', 'TP', 'TN', 'FP', 'FN'])
        for csv in glob.glob(os.path.join(succ_folder, '*.csv')):
            logger.info(csv)
            df = pd.read_csv(csv, sep=',')
            anomaly_t = get_first_anomaly_signal_time(model, df.values[:, 1:], df.values[:, 0].reshape(-1))
            
            stat = {}
            if anomaly_t is not None:
                stat['FP'] = 1
            else:
                stat['TN'] = 1
            stat.update({"sample name": csv})
            stat_df = stat_df.append(stat, ignore_index=True)

        for csv in glob.glob(os.path.join(unsucc_folder, '*', '*.csv')):
            logger.info(csv)
            df = pd.read_csv(csv, sep=',')
            anomaly_label_and_signal_time = pickle.load(open(os.path.join(
                os.path.dirname(csv),
                'anomaly_label_and_signal_time.pkl'
            ), 'r'))
            anomaly_type = anomaly_label_and_signal_time[0]
            anomaly_t_by_human = anomaly_label_and_signal_time[1].to_sec()
            anomaly_t = get_first_anomaly_signal_time(model, df.values[:, 1:], df.values[:, 0].reshape(-1))

            stat = {}
            if anomaly_t is None:
                stat['FN'] = 1
            else:
                t_diff = abs(anomaly_t_by_human-anomaly_t)
                if t_diff > 1:
                    stat['FP'] = 1
                else:
                    stat['TP'] = 1
            stat.update({"sample name": csv, 'anomaly_type': anomaly_type})
            stat_df = stat_df.append(stat, ignore_index=True)


if __name__ == '__main__':
    run()
