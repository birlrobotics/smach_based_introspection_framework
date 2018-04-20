from sklearn.model_selection import train_test_split
from birl_hmm.hmm_training import train_model, hmm_util
import numpy as np
import ipdb

def run(list_of_mat, model_type, model_config, score_metric):
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
    return {
        'model': {
            'hmm_model':best_model,
            'threshold_for_introspection':threshold,
        },
        'model_info': {
            'threshold_for_introspection':threshold,
            'training_report': tried_models,
            'size_of_train_set': len(list_of_train_mat),
            'size_of_test_set': len(list_of_test_mat),
        }
    }
