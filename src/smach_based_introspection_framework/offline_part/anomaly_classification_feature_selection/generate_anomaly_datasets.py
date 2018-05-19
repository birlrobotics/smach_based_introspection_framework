from smach_based_introspection_framework import ExperimentRecord
import glob
from smach_based_introspection_framework._constant import (
    experiment_record_folder, 
    anomaly_classification_feature_selection_folder,
)
import os
from rostopics_to_timeseries import OfflineRostopicsToTimeseries
from ac_feature_schemes import feature_schemes
import itertools
import pandas as pd
import pickle
import shutil
import rospy
import coloredlogs, logging
import ac_variables
coloredlogs.install()

def generate_and_save_csv(output_csv, er, st, et, filtering_scheme, ortt, logger):
    if not os.path.isfile(output_csv):
        try:
            t, mat = ortt.get_timeseries_mat(
                er.rosbag,
                st,
                et,
            )
        except Exception as e:
            logger.error("Fail to get timeseries_mat: %s"%e)
            raise e
        
        df = pd.DataFrame(mat, columns=filtering_scheme.timeseries_header, index=t)
        output_dir = os.path.dirname(output_csv)
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        df.to_csv(output_csv)
        logger.debug("Done.")
    else:
        logger.debug("Already done, gonna skip.")

def setup_and_get_scheme_folder(scheme_count, feature_scheme, exp_dir):
    scheme_folder = os.path.join(
        anomaly_classification_feature_selection_folder,
        'No.%s filtering scheme'%scheme_count,
    )
    if not os.path.isdir(scheme_folder):
        os.makedirs(scheme_folder)

    scheme_info_file = os.path.join(
        scheme_folder,
        "filtering scheme info.txt",
    )
    if not os.path.isfile(scheme_info_file):
        f = open(scheme_info_file, 'w')
        f.write(feature_scheme.detailed_info)
        f.close()
    return scheme_folder

def run():
    logger = logging.getLogger('AcfsGenAnoDataset')
    exp_dirs = [i for i in glob.glob(os.path.join(experiment_record_folder, '*')) if os.path.isdir(i)]
    exp_dirs.sort() 
    for exp_dir in exp_dirs:
        logger.info(exp_dir)
        er = ExperimentRecord(exp_dir)
        for scheme_count, feature_scheme in enumerate(feature_schemes):
            window_size_config_arg = feature_scheme.window_size_config_arg
            filtering_scheme = feature_scheme.filtering_scheme

            ortt = OfflineRostopicsToTimeseries(filtering_scheme) 
            secs_before, secs_after = window_size_config_arg 

            for anomaly_count, (anomaly_type, anomaly_time) in enumerate(er.anomaly_signals):
                logger.info("anomaly_type: %s, anomaly_time: %s"%(anomaly_type, anomaly_time)) 
                output_csv = os.path.join(
                    setup_and_get_scheme_folder(scheme_count, feature_scheme, exp_dir),
                    'anomalies_grouped_by_type',
                    'anomaly_type_(%s)'%anomaly_type,
                    'No.%s anomaly from %s'%(anomaly_count, os.path.basename(exp_dir)),
                    'No.%s anomaly from %s.csv'%(anomaly_count, os.path.basename(exp_dir))
                )
                generate_and_save_csv(output_csv, er, anomaly_time-rospy.Duration(secs_before), anomaly_time+rospy.Duration(secs_after), filtering_scheme, ortt, logger)

                with open(os.path.join(os.path.dirname(output_csv), "anomaly_time.pkl"), 'wb') as f:
                    pickle.dump(anomaly_time, f)

if __name__ == '__main__':
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    run()
