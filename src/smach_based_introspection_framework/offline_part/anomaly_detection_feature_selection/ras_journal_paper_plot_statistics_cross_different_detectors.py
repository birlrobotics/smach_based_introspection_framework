from smach_based_introspection_framework._constant import (
    datasets_of_filtering_schemes_folder, folder_time_fmt
)
import glob
import os
import coloredlogs, logging
from smach_based_introspection_framework.online_part.anomaly_detector import Detectors 
from sklearn.externals import joblib
import pandas as pd
import numpy as np
import pickle
import datetime
import re
from filtering_schemes import filtering_schemes
import ipdb
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.size'] = 8


pd.set_option("display.max_rows", None)
pd.set_option("display.max_columns", None)
pd.set_option("display.width", 1000)
pd.set_option("display.precision", 3)

coloredlogs.install()
logger = logging.getLogger()
logger.setLevel(logging.INFO)
consoleHandler = logging.StreamHandler()
consoleHandler.setLevel(logging.INFO)
logger.addHandler(consoleHandler)

def get_first_anomaly_signal_time(detector, model, timeseries_mat, ts):
    loglik_and_threshold = []
    first_t = None
    for idx, t in enumerate(ts):
        now_skill, anomaly_detected, metric, threshold = detector.add_one_smaple_and_identify_skill_and_detect_anomaly(
                            np.array(timeseries_mat[idx]).reshape(1,-1), now_skill=1)
        loglik_and_threshold.append([metric, threshold])
        if anomaly_detected and first_t is None:
            first_t = t
    return first_t, loglik_and_threshold

def run():
    
    model_folders = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'introspection_models',
        'No.* filtering scheme', 
        'skill *',
    ))
    if len(model_folders) == 0:
        logger.error('Without any introspection model')
        
    for model_folder in model_folders:
        logger.info(os.path.realpath(model_folder))
        model = joblib.load(os.path.join(model_folder, 'introspection_model'))

        avaliable_anomaly_detectors=[
                Detectors.DetectorBasedOnLogLikByHiddenState(
                {1: model['hmm_model']}, 
                {1: model['zhat_loglik_threshold_by_max_min_dict']},) #zhat_loglik_threshold_by_mean_std_dict
                ,
                Detectors.DetectorBasedOnGradientOfLoglikCurve(
                    {1: model['hmm_model']}, 
                    {1: model['threshold_for_introspection']},)
            ]
            
        path_postfix = os.path.relpath(model_folder,
                                   os.path.join(datasets_of_filtering_schemes_folder, 'introspection_models'))
        succ_folder = os.path.join(datasets_of_filtering_schemes_folder,
                                   path_postfix).replace(os.sep+"skill", os.sep+"successful_skills"+os.sep+"skill")
        unsucc_folder = os.path.join(datasets_of_filtering_schemes_folder,
                                   path_postfix).replace(os.sep+"skill", os.sep+"unsuccessful_skills"+os.sep+"skill")

        stat_df = pd.DataFrame(columns=['detector','sample_name', 'anomaly_type', 'TP', 'TN', 'FP', 'FN'])            
            
        succ_csvs   = glob.glob(os.path.join(succ_folder, '*', '*.csv'))
        if len(succ_csvs) == 0:
            logger.error('without the successful recordings of %s for testing, gonna skip'%path_postfix)
            continue
        for i, csv in enumerate(succ_csvs):
            logger.info(csv)
            df = pd.read_csv(csv, sep=',')
            for detector in avaliable_anomaly_detectors:
                first_anomaly_t, loglik_and_threshold = get_first_anomaly_signal_time(detector, model,
                                                                                      df.values[:, 1:], df.values[:, 0].reshape(-1))
                stat = {}
                if first_anomaly_t is not None:
                    stat['FP'] = 1
                else:
                    stat['TN'] = 1
                stat.update({"detector":detector.__name__, "sample_name": os.path.basename(csv),'anomaly_type': 'success'})
                stat_df = stat_df.append(stat, ignore_index=True)
            logger.warning("Finish testing:%s"%csv)

        unsucc_csvs = glob.glob(os.path.join(unsucc_folder, '*', '*.csv'))
        if len(unsucc_csvs) == 0:
            logger.error('without the unsuccessful recordings of %s for testing, gonna skip'%path_postfix)
            continue
        for i, csv in enumerate(unsucc_csvs):
            logger.info(csv)
            df = pd.read_csv(csv, sep=',')
            anomaly_label_and_signal_time = pickle.load(open(os.path.join(
                os.path.dirname(csv), 'anomaly_label_and_signal_time.pkl'), 'r'))
            anomaly_type = anomaly_label_and_signal_time[0]
            anomaly_t_by_human = anomaly_label_and_signal_time[1].to_sec()
            for detector in avaliable_anomaly_detectors:
                first_anomaly_t, loglik_and_threshold = get_first_anomaly_signal_time(detector, model,
                                                                                      df.values[:, 1:], df.values[:, 0].reshape(-1))
                stat = {}
                if first_anomaly_t is None:
                    stat['FN'] = 1
                else:
                    if anomaly_type != 'no_object':
                        t_diff = abs(anomaly_t_by_human - first_anomaly_t)
                        if t_diff > 1:
                            stat['FP'] = 1
                        else:
                            stat['TP'] = 1
                    else:
                        stat['TP'] = 1
                stat.update({"detector":detector.__name__, "sample_name": os.path.basename(csv), 'anomaly_type': anomaly_type})
                stat_df = stat_df.append(stat, ignore_index=True)
            logger.warning("Finish testing:%s"%csv)

        for detector in avaliable_anomaly_detectors:
            output_dir = os.path.join(
                datasets_of_filtering_schemes_folder,
                'ras_journal_paper_plots',
                detector.__name__,
                path_postfix,)
            if not os.path.isdir(output_dir):
                os.makedirs(output_dir)
            detector_file = os.path.join(output_dir, os.path.basename(output_dir)+' stat.csv')
            if os.path.isfile(detector_file):
                logger.warning("Stat file already exists, rewrite the existing file")
            df_detector = stat_df.loc[stat_df['detector']==detector.__name__]
            df_detector.to_csv(detector_file)

    for detector in avaliable_anomaly_detectors:
        generate_human_readable_report(detector)

def generate_human_readable_report(detector=None):
    stat_files = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'ras_journal_paper_plots',
        detector.__name__,
        'No.* filtering scheme',
        'skill *',
        '*.csv',
    ))
    prog = re.compile(r'No.(\d+) filtering scheme%sskill (\d+)'%os.sep)
    big_df = pd.DataFrame()
    for csv in stat_files:
        logger.info(csv)

        m = prog.search(csv)
        if not m:
            raise Exception("Fail to extract scheme no and skill no from %s"%csv)

        scheme_no = m.group(1)
        skill_no = m.group(2)

        df = pd.read_csv(csv) 

        df['scheme_no'] = scheme_no
        df['skill_no'] = skill_no
        df = df.fillna({"anomaly_type": "success"}).fillna(0)
    
        big_df = big_df.append(df)
    big_df = big_df.drop([ big_df.columns[0], 'detector', 'sample_name'], axis=1).set_index(['scheme_no', 'skill_no', 'anomaly_type'])
    big_df = big_df.astype(int)

    report = open(os.path.join(datasets_of_filtering_schemes_folder, detector.__name__+'_report.txt'), 'w') 
    report.write("scheme info\n")
    report.write("="*30+"\n")
    for scheme_count, filtering_scheme in enumerate(filtering_schemes):
        report.write("scheme %s:\n"%scheme_count)
        report.write("-"*30+"\n")
        report.write(str(filtering_scheme.timeseries_header))
        report.write("\n\n")

    report.write("scheme performance\n")
    report.write("="*30+"\n")
    report.write("granularity: scheme\n")
    report.write("-"*30+"\n")
    report.write(str(append_metrics(big_df.groupby(level=[0]).sum())))
    report.write("\n\n")
    report.write("granularity: skill\n")
    report.write("-"*30+"\n")
    report.write(str(append_metrics(big_df.groupby(level=[0, 1]).sum())))
    report.write("\n\n")
    report.write("granularity: anomaly type\n")
    report.write("-"*30+"\n")
    report.write(str(append_metrics(big_df.groupby(level=[0, 1, 2]).sum())))
    report.write("\n\n")
    return big_df

def append_metrics(df_):
    df = df_.copy()
    df['precision'] = df['TP']/(df['TP']+df['FP']) 
    df['recall'] = df['TP']/(df['TP']+df['FN']) 
    df['F1score'] = 2*df['TP']/(2*df['TP']+df['FP']+df['FN'])

    df['accuracy'] = (df['TP']+df['TN'])/(df['TP']+df['TN']+df['FP']+df['FN']) 
    return df





            
def plot_the_results():
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = Axes3D(fig)
    # detector-1
    for z in [0, 10, 20, 30, 40, 50, 60]:
        xs = np.array([1, 4, 7, 10])
        ys = np.random.rand(4)
        for modifier in range(-45,45,5):
            ax.bar(xs, ys, zs=z + (float(modifier)/100), zdir='y', color='r',alpha=0.5)
    # detector-2
    for z in [0, 10, 20, 30, 40, 50, 60]:
        xs = np.array([2, 5, 8, 11])
        ys = np.random.rand(4)
        for modifier in range(-45,45,5):
            ax.bar(xs, ys, zs=z + (float(modifier)/100), zdir='y', color='b',alpha=0.5)
    ax.set_xlabel('Metrics')
    ax.set_ylabel('Skill')
    ax.set_zlabel('Value')
    plt.xticks([1.5, 4.5, 7.5, 10.5], ['precision', 'recall', 'f1score', 'accuracy'])
    plt.yticks([0, 10, 20, 30, 40, 50, 60], ['average', '3', '4', '5', '7', '8', '9'])
    red_proxy = plt.Rectangle((0, 0), 1, 1, fc="r")
    blue_proxy = plt.Rectangle((0, 0), 1, 1, fc="b")
    ax.legend([blue_proxy,red_proxy],['Gradient-based','Hidden State-based'])
    plt.show()
        
if __name__ == '__main__':
    run()
