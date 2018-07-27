from sklearn.model_selection import train_test_split
from birl_hmm.hmm_training import train_model, hmm_util
import numpy as np
import pandas as pd
import ipdb

def run(list_of_mat, model_type, model_config, score_metric, logger=None):
    list_of_train_mat, list_of_test_mat = train_test_split(list_of_mat, test_size=0.25)
    
    if len(list_of_train_mat) == 0:
        raise Exception('Contains 0 train samples, failed to train introspection model')
    else:
        print 'Contains %s train samples'%(len(list_of_train_mat))
        

    if len(list_of_test_mat) == 0:
        raise Exception('Contains 0 test samples, failed to train introspection model')
    else:
        print 'Contains %s test samples'%(len(list_of_test_mat))
    best_model, test_score, tried_models = train_model.run(
        list_of_train_mat=list_of_train_mat, 
        list_of_test_mat=list_of_test_mat,
        model_type=model_type,
        model_config=model_config,
        score_metric=score_metric,
    )


    list_of_log_curves = [hmm_util.fast_log_curve_calculation(i, best_model) for i in list_of_test_mat]
    max_candidate = []
    min_candidate = []
    for curve in list_of_log_curves:
        arr = np.array(curve)
        gradient_curve =  arr[1:]-arr[:-1]
        max_candidate.append(max(gradient_curve))
        min_candidate.append(min(gradient_curve))

    min_gradient = min(min_candidate)
    max_gradient = max(max_candidate)
    gradient_range = max_gradient-min_gradient
    threshold = min_gradient-gradient_range/2
    loglik_threshold_by_zhat_dict = get_loglik_threshold_with_the_same_zhat(best_model, list_of_mat, logger=logger)
    return {
        'model': {
            'hmm_model':best_model,
            'threshold_for_introspection':threshold,
            'loglik_threshold_by_zhat_dict':loglik_threshold_by_zhat_dict,
        },
        'model_info': {
            'test_score': test_score,
            'threshold_for_introspection':threshold,
            'loglik_threshold_by_zhat_dict':loglik_threshold_by_zhat_dict,            
            'training_report': tried_models,
            'size_of_train_set': len(list_of_train_mat),
            'size_of_test_set': len(list_of_test_mat),
        }
    }

def get_loglik_threshold_with_the_same_zhat(model, list_of_mat, logger=None):
    import hmmlearn
    zhat_log = pd.DataFrame()
    state_sequences = []
    if issubclass(type(model), hmmlearn.hmm._BaseHMM):
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
    else:
        logger.error('Only for the hmmlean models')
        return None
    loglik_threshold_by_zhat_dict = {}        
    for iz in sorted(zhat_log['zhat'].unique().tolist()):
        zlog = zhat_log['log'].loc[zhat_log['zhat'] == iz].values
        zlog = filter_the_outliers(zlog,logger=logger)
        zlog_min = zlog.min()
        zlog_max = zlog.max()
        zlog_mean = np.mean(zlog)
        zlog_var  = np.var(zlog)
        #threshold = zlog_min - (zlog_max - zlog_min)/2
        threshold = zlog_mean - 2.0 * zlog_var        
        loglik_threshold_by_zhat_dict[iz] = threshold
    show_logliks_with_the_same_hidden_state(zhat_log,list_of_mat,logger=logger)
    show_hidden_state_sequences(zhat_log, state_sequences)
    return loglik_threshold_by_zhat_dict

def filter_the_outliers(zlog, logger=None):
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
    logger.warning('Filtered %s outliters'%outliter_idx.shape[-1])
    return zlog
