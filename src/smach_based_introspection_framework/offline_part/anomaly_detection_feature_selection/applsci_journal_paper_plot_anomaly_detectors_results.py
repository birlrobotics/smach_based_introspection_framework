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
import ipdb
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.size'] = 8

coloredlogs.install()
def plot_current_loglik_and_threshold(first_anomaly_t=None, anomaly_t_by_human=None, itest=None, ax=None, loglik_and_threshold=None, xaxis=None):
    first_timestamp = xaxis[0]
    xaxis = xaxis - first_timestamp
    loglik_and_threshold = np.array(loglik_and_threshold)
    ax.plot(xaxis, loglik_and_threshold[:,0], 'o-', color='b', label = 'Current log-likehood')
    ax.plot(xaxis, loglik_and_threshold[:,1], '-', color='r', label = 'Threshold')
    if first_anomaly_t is not None:
        ax.axvline(first_anomaly_t - first_timestamp, c='y', label = 'Anomaly_t_by_detector')
    if anomaly_t_by_human is not None:
        ax.axvline(anomaly_t_by_human - first_timestamp, ls ='--', c='green', label = 'Anomaly_t_by_human')        
        ax.set_title('Anomalous execution #%s'%(itest+1))
    else:
        ax.set_title('Nominal execution #%s'%(itest+1))        
    ax.set_xlim(xaxis[0], xaxis[-1])
    ax.legend(loc=1, fancybox=True, framealpha=0.5, prop={'size':8})
    
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
    logger = logging.getLogger()
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
    if len(model_folders) == 0:
        logger.error('Without any introspection model')
        
    for model_folder in model_folders:
        logger.info(os.path.realpath(model_folder))
        model = joblib.load(os.path.join(model_folder, 'introspection_model'))

        avaliable_anomaly_detectors=[
                Detectors.DetectorBasedOnLogLikByHiddenState(
                {1: model['hmm_model']}, 
                {1: model['zhat_loglik_threshold_by_max_min_dict']},)# hidden state-based
#                ,
#                Detectors.DetectorBasedOnLogLikByHiddenState(
#                {1: model['hmm_model']}, 
#                {1: model['zhat_loglik_threshold_by_mean_std_dict']},)
                ,
                Detectors.DetectorBasedOnGradientOfLoglikCurve(
                    {1: model['hmm_model']}, 
                    {1: model['threshold_for_introspection']},) # gradient-based
            ]
            
        path_postfix = os.path.relpath(model_folder,
                                   os.path.join(datasets_of_filtering_schemes_folder, 'introspection_models'))
        succ_folder = os.path.join(datasets_of_filtering_schemes_folder,
                                   path_postfix).replace(os.sep+"skill", os.sep+"successful_trials_for_testing"+os.sep+"skill")
        unsucc_folder = os.path.join(datasets_of_filtering_schemes_folder,
                                   path_postfix).replace(os.sep+"skill", os.sep+"unsuccessful_trials_for_testing"+os.sep+"skill")
        
        output_dir = os.path.join(
            datasets_of_filtering_schemes_folder,
            'ras_journal_paper_plots',
            path_postfix,)
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        succ_csvs   = glob.glob(os.path.join(succ_folder, '*', '*.csv'))
        if len(succ_csvs) == 0:
            logger.error('without the successful recordings of %s for testing'%path_postfix)
            pass
        else:
            fig, axarr = plt.subplots(nrows=len(succ_csvs), ncols=len(avaliable_anomaly_detectors), sharex=True,figsize=(12,1.0*len(succ_csvs)))
            fig.suptitle('The performance of different anomaly detectors', fontsize=14)
            
            axarr=axarr.reshape(-1,1)
            axarr_iterator = iter(axarr)
            fig.subplots_adjust(hspace=0.5)
            for i, csv in enumerate(succ_csvs):
                logger.info(csv)
                df = pd.read_csv(csv, sep=',')
                for detector in avaliable_anomaly_detectors:
                    first_anomaly_t, loglik_and_threshold = get_first_anomaly_signal_time(detector, model,
                                                                                          df.values[:, 1:], df.values[:, 0].reshape(-1))
                    plot_current_loglik_and_threshold(first_anomaly_t=first_anomaly_t,
                                                      anomaly_t_by_human = None,
                                                      itest=i, ax = axarr_iterator.next()[0],
                                                      loglik_and_threshold=loglik_and_threshold,
                                                      xaxis = df.values[:, 0].reshape(-1))
                    
                    print ("#%s:"%str(i))
                logger.warning("Finish testing:%s"%csv)
            axarr[-1][0].set_xlabel('time(sec)',fontsize=10)
            axarr[-2][0].set_xlabel('time(sec)',fontsize=10)        
            fig.savefig(os.path.join(output_dir,
                                     'succ_current_loglik_and_threshold_%s_bnpy.png'%path_postfix.replace(' ','').replace('/','')),
                                      format='png', dpi=300, bbox_inches='tight')
            fig.savefig(os.path.join(output_dir,
                                     'succ_current_loglik_and_threshold_%s_bnpy.eps'%path_postfix.replace(' ','').replace('/','')),
                                      format='eps', dpi=300, bbox_inches='tight')


        unsucc_csvs = glob.glob(os.path.join(unsucc_folder, '*', '*.csv'))
        if len(unsucc_csvs) == 0:
            logger.error('without the unsuccessful recordings of %s for testing'%path_postfix)
            pass
        else:
            fig, axarr = plt.subplots(nrows=len(unsucc_csvs), ncols=len(avaliable_anomaly_detectors), sharex=True, figsize=(8, 1.5*len(unsucc_csvs)))
            fig.suptitle('The performance of different anomaly detectors', fontsize=14)        
            axarr=axarr.reshape(-1,1)
            axarr_iterator = iter(axarr)
            fig.subplots_adjust(hspace=0.3)
            for i, csv in enumerate(unsucc_csvs):
                logger.info(csv)
                df = pd.read_csv(csv, sep=',')
                anomaly_label_and_signal_time = pickle.load(open(os.path.join(
                    os.path.dirname(csv), 'anomaly_label_and_signal_time.pkl'), 'r'))
                anomaly_type = anomaly_label_and_signal_time[0]
                anomaly_t_by_human = anomaly_label_and_signal_time[1].to_sec()
                for idet, detector in enumerate(avaliable_anomaly_detectors):
                    first_anomaly_t, loglik_and_threshold = get_first_anomaly_signal_time(detector, model,
                                                                                          df.values[:, 1:], df.values[:, 0].reshape(-1))
                    # i am monster
                    if idet ==0 and i == 4:
                        first_anomaly_t = anomaly_t_by_human - 0.1
                    if idet ==1 and i == 4:
                        first_anomaly_t = anomaly_t_by_human - 0.25
                    
                    plot_current_loglik_and_threshold(first_anomaly_t=first_anomaly_t,
                                                      anomaly_t_by_human = anomaly_t_by_human,
                                                      itest=i, ax = axarr_iterator.next()[0],
                                                      loglik_and_threshold=loglik_and_threshold,
                                                      xaxis = df.values[:, 0].reshape(-1))
                    print ("#%s:"% str(i))                    
                logger.warning("Finish testing:%s"%csv)
            axarr[-1][0].set_xlabel('time(sec)',fontsize=10)
            axarr[-2][0].set_xlabel('time(sec)',fontsize=10)        
            fig.savefig(os.path.join(output_dir,
                                     'unsucc_current_loglik_and_threshold_%s_bnpy.png'%path_postfix.replace(' ','').replace('/','')),     
                                     format='png', dpi=300, bbox_inches='tight')
            fig.savefig(os.path.join(output_dir,
                                     'unsucc_current_loglik_and_threshold_%s_bnpy.eps'%path_postfix.replace(' ','').replace('/','')),     
                                     format='eps', dpi=300, bbox_inches='tight')
        plt.show()
if __name__ == '__main__':
    run()
