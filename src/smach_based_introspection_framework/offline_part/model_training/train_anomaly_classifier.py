from sklearn.model_selection import train_test_split
from birl_hmm.hmm_training import train_model, hmm_util
import numpy as np

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

    sorted_model_list = train_model.run(
        list_of_train_mat=list_of_train_mat, 
        list_of_test_mat=list_of_test_mat,
        model_type=model_type,
        model_config=model_config,
        score_metric=score_metric,
    )
    best = sorted_model_list[0]
    list_of_loklik = [best['model'].score(i) for i in list_of_test_mat]
    mat = np.matrix(list_of_loklik)
    mean = mat.mean()
    std = mat.std()
    threshold = mean-2*std

    train_report = [{hmm_util.get_model_config_id(i['now_model_config']): i['score']} for i in sorted_model_list]
    return {
        'model': {
            'hmm_model':best['model'],
            'threshold_for_classification':threshold,
        },
        'model_info': {
            'config_of_best_model': best['now_model_config'],
            'threshold_for_classification':threshold,
            'test_set_log_likelihood_mean':mean,
            'test_set_log_likelihood_std':std,
            'train_report': train_report,
            'size_of_train_set': len(list_of_train_mat),
            'size_of_test_set': len(list_of_test_mat),
        }
    }
