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
    
def run():
    logger = logging.getLogger('FilteringRecordsWithSchemes')
    exp_dirs = [i for i in glob.glob(os.path.join(experiment_record_folder, '*')) if os.path.isdir(i)]
    exp_dirs.sort() 
    for exp_dir in exp_dirs:
        logger.info(exp_dir)
        er = ExperimentRecord(exp_dir)
        for scheme_count, filtering_scheme in enumerate(filtering_schemes):
            ortt = OfflineRostopicsToTimeseries(filtering_scheme) 
            for skill_count, (tag, (st, et)) in enumerate(er.successful_tag_ranges):
                logger.debug("No.%s successful skill"%skill_count)
                output_csv = os.path.join(
                    setup_and_get_scheme_folder(scheme_count, filtering_scheme, exp_dir),
                    'successful_skills',
                    'skill %s'%tag,
                    'No.%s_successful_skill_from_%s'%(skill_count, os.path.basename(exp_dir)),
                    'No.%s_successful_skill_from_%s.csv'%(skill_count, os.path.basename(exp_dir))
                )
                generate_and_save_csv(output_csv, er, st, et, filtering_scheme, ortt, logger)

            for skill_count, (tag, (st, et)) in enumerate(er.unsuccessful_tag_ranges):
                logger.debug("No.%s unsuccessful skill"%skill_count)
                output_csv = os.path.join(
                    setup_and_get_scheme_folder(scheme_count, filtering_scheme, exp_dir),
                    'unsuccessful_skills',
                    'skill %s'%tag,
                    'No.%s_unsuccessful_skill_from_%s'%(skill_count, os.path.basename(exp_dir)),
                    'No.%s_unsuccessful_skill_from_%s.csv'%(skill_count, os.path.basename(exp_dir))
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

            logger.debug("whole experiment")
            output_csv = os.path.join(
                setup_and_get_scheme_folder(scheme_count, filtering_scheme, exp_dir),
                'whole_experiment',
                '%s'%(os.path.basename(exp_dir)),
                '%s.csv'%(os.path.basename(exp_dir)),
            )
            if os.path.exists(output_csv):
                logger.info(output_csv)
                continue
            generate_and_save_csv(output_csv, er, None, None, filtering_scheme, ortt, logger)
            with open(os.path.join(os.path.dirname(output_csv), "tag_ranges.pkl"), 'wb') as f:
                pickle.dump(er.tag_ranges, f)
            try:
                with open(os.path.join(os.path.dirname(output_csv), "anomaly_signals.pkl"), 'wb') as f:
                    pickle.dump(er.anomaly_signals, f)
            except:
                logger.info("No anomalies")


if __name__ == '__main__':
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.DEBUG)
    logger.addHandler(consoleHandler)
    run()
