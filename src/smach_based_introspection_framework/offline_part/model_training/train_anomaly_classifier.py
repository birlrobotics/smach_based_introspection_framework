from sklearn.model_selection import train_test_split
from birl_hmm.hmm_training import train_model, hmm_util
import numpy as np
from scipy.stats import norm
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

    list_of_loklik = [best_model.score(i) for i in list_of_mat]
    mean_and_std_of_the_norm_distribution = norm.fit(list_of_loklik)
    return {
        'model': {
            'hmm_model':best_model,
            'mean_and_std_of_the_norm_distribution': mean_and_std_of_the_norm_distribution,
        },
        'model_info': {
            'test_score': test_score,
            'training_report': tried_models,
            'size_of_train_set': len(list_of_train_mat),
            'size_of_test_set': len(list_of_test_mat),
            'mean_and_std_of_the_norm_distribution': mean_and_std_of_the_norm_distribution,
            'loklik of all input mat': list_of_loklik,
        }
    }
