
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

    model_folders = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'introspection_models',
        'No.* filtering scheme', 
        'skill *',
    ))

    for model_folder in model_folders:
        logger.info(os.path.realpath(model_folder))
        model = joblib.load(os.path.join(model_folder, 'introspection_model'))
        path_postfix = os.path.relpath(model_folder, os.path.join(datasets_of_filtering_schemes_folder, 'introspection_models'))

        output_dir = os.path.join(
            datasets_of_filtering_schemes_folder,
            'introspection_statistics',
            path_postfix,
        )
        stat_file = os.path.join(output_dir, os.path.basename(output_dir)+' stat.csv')
        if os.path.isfile(stat_file):
            logger.info("Stat file already exists, gonna skip")
            continue

        succ_folder = os.path.join(datasets_of_filtering_schemes_folder, path_postfix).replace(os.sep+"skill", os.sep+"successful_skills"+os.sep+"skill")
        unsucc_folder = os.path.join(datasets_of_filtering_schemes_folder, path_postfix).replace(os.sep+"skill", os.sep+"unsuccessful_skills"+os.sep+"skill")
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
            stat.update({"sample name": os.path.basename(csv)})
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
            stat.update({"sample name": os.path.basename(csv), 'anomaly type': anomaly_type})
            stat_df = stat_df.append(stat, ignore_index=True)

        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        stat_df.to_csv(stat_file)

if __name__ == '__main__':
    run()
