from smach_based_introspection_framework import ExperimentRecord
import glob
from smach_based_introspection_framework._constant import (
    experiment_record_folder, 
    datasets_of_filtering_schemes_folder,
)
import os
from rostopics_to_timeseries import OfflineRostopicsToTimeseries
from filtering_schemes import filtering_schemes
import itertools
import pandas as pd
import pickle
import shutil
import rospy
import coloredlogs, logging
coloredlogs.install()

def setup_and_get_scheme_folder(scheme_count, filtering_scheme, exp_dir):
    scheme_folder = os.path.join(
        datasets_of_filtering_schemes_folder,
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
        f.write(filtering_scheme.info)
        f.close()
    return scheme_folder

def generate_and_save_csv(output_csv, er, st, et, filtering_scheme, ortt, logger):
    if not os.path.isfile(output_csv):
        t, mat = ortt.get_timeseries_mat(
            er.rosbag,
            st,
            et,
        )
        df = pd.DataFrame(mat, columns=filtering_scheme.timeseries_header, index=t)
        output_dir = os.path.dirname(output_csv)
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        df.to_csv(output_csv)
        logger.debug("Done.")
    else:
        logger.debug("Already done, gonna skip.")
    
def run():
    logger = logging.getLogger('FilteringRecordsWithSchemes')
    logger.setLevel(logging.INFO)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.INFO)
    logger.addHandler(consoleHandler)
    exp_dirs = [i for i in glob.glob(os.path.join(experiment_record_folder, '*')) if os.path.isdir(i)]
    exp_dirs.sort() 
    for scheme_count, filtering_scheme in enumerate(filtering_schemes):
        for exp_dir in exp_dirs:
            logger.info(exp_dir)
            er = ExperimentRecord(exp_dir)
            ortt = OfflineRostopicsToTimeseries(filtering_scheme, rate=10) 
            for skill_count, (tag, (st, et)) in enumerate(er.successful_tag_ranges):
                logger.debug("No.%s successful skill"%skill_count)
                output_csv = os.path.join(
                    setup_and_get_scheme_folder(scheme_count, filtering_scheme, exp_dir),
                    'successful_skills',
                    'skill %s'%tag,
                    'No.%s successful skill from %s.csv'%(skill_count, os.path.basename(exp_dir))
                )
                generate_and_save_csv(output_csv, er, st, et, filtering_scheme, ortt, logger)

            for skill_count, (tag, (st, et)) in enumerate(er.unsuccessful_tag_ranges):
                logger.debug("No.%s unsuccessful skill"%skill_count)
                output_csv = os.path.join(
                    setup_and_get_scheme_folder(scheme_count, filtering_scheme, exp_dir),
                    'unsuccessful_skills',
                    'skill %s'%tag,
                    'No.%s unsuccessful skill from %s'%(skill_count, os.path.basename(exp_dir)),
                    'No.%s unsuccessful skill from %s.csv'%(skill_count, os.path.basename(exp_dir))
                )
                try:
                    generate_and_save_csv(output_csv, er, st, et+rospy.Duration(2), filtering_scheme, ortt, logger)
                    pickle.dump(
                        er.anomaly_signals[skill_count],
                        open(os.path.join(
                            os.path.dirname(output_csv),
                            'anomaly_label_and_signal_time.pkl'
                        ), 'w'),
                    )
                except Exception as e:
                    logger.error("Exception: %s"%e)
                    shutil.rmtree(os.path.dirname(output_csv), ignore_errors=True) 
                    break

if __name__ == '__main__':
    run()