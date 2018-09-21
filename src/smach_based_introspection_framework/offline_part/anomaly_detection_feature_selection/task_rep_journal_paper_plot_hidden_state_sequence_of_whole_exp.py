from smach_based_introspection_framework._constant import (
    datasets_of_filtering_schemes_folder, folder_time_fmt
)
import glob
import os
import pickle
import datetime
import pandas as pd
import numpy as np
import coloredlogs, logging
from sklearn.externals import joblib
from birl_hmm.hmm_training import train_model, hmm_util
from smach_based_introspection_framework.online_part.anomaly_detector import Detectors
import birl_hmm
import hmmlearn
import matplotlib.pyplot as plt
import matplotlib
import ipdb
matplotlib.rcParams.update({'font.size': 12})

coloredlogs.install()

THRESHOLDING_BY_MEAN_AND_STD = False

def run():
    logger = logging.getLogger('ras_paper_plots')
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
        path_postfix = os.path.relpath(model_folder, os.path.join(datasets_of_filtering_schemes_folder, 'introspection_models'))
        succ_folder = os.path.join(datasets_of_filtering_schemes_folder,
                                   path_postfix).replace(os.sep+"skill", os.sep+"successful_skills"+os.sep+"skill")
        logger.info(succ_folder)
        csvs = glob.glob(os.path.join(
            succ_folder,
            '*', '*.csv',
        ))
        if len(csvs) == 0:
            logger.warning('without the successful data of %s. gonna to skip' %path_postfix)
            continue
        
        list_of_mat = []
        for j in csvs:
            df = pd.read_csv(j, sep=',')
            # Exclude 1st column which is time index
            list_of_mat.append(df.values[:, 1:])

        output_dir = os.path.join(
            datasets_of_filtering_schemes_folder,
            'ras_journal_paper_plots',
            path_postfix,)
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        show_logliks_with_the_same_hidden_state(model['hmm_model'], list_of_mat, logger=logger, output_dir=output_dir)

def show_logliks_with_the_same_hidden_state(model, list_of_mat, logger=None, output_dir=None):
    colors  = ['r', 'g', 'b', 'g', 'c', 'm', 'y', 'k']
    markers = ['o', '+', '*', 's', 'x', '>', '<', '.']
    
    zhat_log = pd.DataFrame()
    state_sequences = []
    if issubclass(type(model), birl_hmm.bnpy_hmm_wrapper.hmm.HongminHMM):
        list_of_log_curves = [model.calc_log(i) for i in list_of_mat]
        curve_diff = []
        zhat = []
        for i, curve in enumerate(list_of_log_curves):
            curve_diff += [curve[0]] + np.diff(curve).tolist()
            state_sequence = model.predict(list_of_mat[i])
            zhat += state_sequence.tolist()
            state_sequences.append(state_sequence)
        zhat_log['zhat'] = zhat
        zhat_log['log'] = curve_diff
        model.n_components = model.model.obsModel.K
        
    else:
        logger.info('Only for the hmmlean models')
        list_of_log_curves = [hmm_util.fast_log_curve_calculation(i, model) for i in list_of_mat]
        curve_diff = []
        zhat = []
        for i, curve in enumerate(list_of_log_curves):
            curve_diff += [curve[0]] + np.diff(curve).tolist()
            _, state_sequence = model.decode(list_of_mat[i])
            zhat += state_sequence.tolist()
            state_sequences.append(state_sequence)
        zhat_log['zhat'] = zhat
        zhat_log['log'] = curve_diff
    
    fig, axarr = plt.subplots(nrows=len(zhat_log['zhat'].unique().tolist()), ncols=1, sharex=True)
    plt.subplots_adjust(hspace=0.4)
    for i, iz in enumerate(sorted(zhat_log['zhat'].unique().tolist())):
        axarr[i].plot(zhat_log['log'].loc[zhat_log['zhat'] == iz].values,
                      marker = markers[i], color = colors[i], linestyle = 'None', )
        zlog = zhat_log['log'].loc[zhat_log['zhat'] == iz].values
        zlog = filter_the_outliers(i, zlog, logger=logger)
        zlog_min = zlog.min()
        zlog_max = zlog.max()
        zlog_mean = np.mean(zlog)
        zlog_var  = np.var(zlog)
        if THRESHOLDING_BY_MEAN_AND_STD: 
            threshold = zlog_mean - 2.0 * zlog_var
        else:
            threshold = zlog_min - (zlog_max - zlog_min)/2
        axarr[i].axhline(threshold, color = 'r', linewidth=2, label='Threshold') 
        axarr[i].axhline(zlog_mean, linestyle='--', color = 'black', linewidth=2, label='Mean')       
        axarr[i].set_title('All the correspounding log-likelihood values of zhat={0}'.format(iz))
        axarr[i].legend(loc=1,fancybox=True, framealpha=0.5, prop={'size':8})
    axarr[-1].set_xlabel('Index')    
    plt.savefig(os.path.join(output_dir, 'logliks_with_the_same_hidden_state.png'), format='png',dpi=300)
    plt.savefig(os.path.join(output_dir, 'logliks_with_the_same_hidden_state.eps'), format='eps',dpi=300)
    
    show_hidden_state_sequences(K=model.n_components, zseqs=state_sequences, logger=logger, output_dir=output_dir)
    
def show_hidden_state_sequences(K=None, zseqs=None, logger=None, output_dir=None):
    logger.warning('plotting the hidden state sequences')
    nseqs = len(zseqs)
    fig, axarr = plt.subplots(nrows=nseqs,ncols=1, sharex=True, )
    axarr = np.atleast_1d(axarr).flatten().tolist()
    z_img_height = 2
    top=0.95
    bottom=0.1
    left=0.20
    right=0.9
    hspace=0.25
    wspace=0.01
    for i, zseq in enumerate(zseqs):
        z_img_cmp = matplotlib.cm.get_cmap('Set1',K)
        img_TD = np.tile(zseq, (z_img_height, 1))
        axarr[i].imshow(img_TD, interpolation='nearest',
                  vmin=-0.5, vmax=(K-1)+0.5,cmap=z_img_cmp)
        axarr[i].set_yticks([])
    plt.subplots_adjust(top=top, bottom=bottom,
                        left=left,right=right,hspace=hspace, wspace=wspace)
    ax_handle0 = axarr[0]
    ax_handle1 = axarr[-1]    
    bbox0 =ax_handle0.get_position() 
    bbox1 =ax_handle1.get_position()
    width = (1.0 - bbox0.x1)/2
    height = bbox0.y1 - bbox1.y0
    cax = fig.add_axes([bbox1.x1+0.01, bbox1.y0, width, height])
    cbbox_h = fig.colorbar(ax_handle0.images[0],cax=cax, orientation='vertical')
    cbbox_h.set_ticks(np.arange(K))
    cbbox_h.set_ticklabels(np.arange(K))
    cbbox_h.ax.tick_params()
    axarr[-1].set_xlabel('time(0.1s)')
    plt.savefig(os.path.join(output_dir, 'hidden_state_sequences.png'), format='png',dpi=300)    
    plt.savefig(os.path.join(output_dir, 'hidden_state_sequences.eps'), format='eps',dpi=300)
    
def filter_the_outliers(i, zlog, logger=None):
    z = zlog.copy()
    threshold = 100
    diff = np.abs(z - np.median(z))
    median_diff = np.median(diff)
    if median_diff == 0:
        s = 0
    else:
        s = diff / float(median_diff)
    mask = s > threshold
    z[mask] = np.median(z)
    outliter_idx = np.where(zlog != z)[0]
    zlog = np.delete(zlog, outliter_idx)
    logger.warning('Filtered %s outliters of zhat=%s'%(outliter_idx.shape[-1], i))
    return zlog

if __name__=='__main__':
    run()
