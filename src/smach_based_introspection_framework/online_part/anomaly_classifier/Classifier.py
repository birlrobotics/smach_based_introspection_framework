

class BaseClassifier(object):
    def __init__(self, model_group_by_anomaly_type):
        if len(model_group_by_anomaly_type) == 0:
            raise Exception("Empty model_group_by_anomaly_type")
        self.model_group_by_anomaly_type = model_group_by_anomaly_type

    def predict(self, mat):
        raise Exception("Unimplemented")

    def predict_proba(self, mat):
        raise Exception("Unimplemented")


class NormalDistributedConfidenceClassifier(BaseClassifier):
    def __init__(self, model_group_by_anomaly_type):
        super(NormalDistributedConfidenceClassifier, self).__init__(model_group_by_anomaly_type)

        for k, v in self.model_group_by_anomaly_type.iteritems():
            if type(v) is not dict:
                raise Exception("anomaly type \"%s\"'s model is not of type dict")
            
            if v.has_key('mean_and_std_of_the_norm_distribution') is not True:
                raise Exception("anomaly type \"%s\"'s model does NOT contain \"mean_and_std_of_the_norm_distribution\" key")


    def predict(self, mat):
        ret = []                                                               
        for anomaly_type, model in self.model_group_by_anomaly_type.iteritems():
            hmm_model    = model['hmm_model']                                  
            score      = hmm_model.score(mat)
            ret.append((anomaly_type, score))

        ret = sorted(ret, key=lambda x: x[1])
        return ret[-1][0]

    def predict_proba(self, mat):
        from scipy.stats import norm

        ret = []                                                               
        for anomaly_type, model in self.model_group_by_anomaly_type.iteritems():
            hmm_model    = model['hmm_model']                                  
            mean_and_std = model['mean_and_std_of_the_norm_distribution']      
            score      = hmm_model.score(mat)
            confidence = norm.cdf(score, mean_and_std[0] - 3 * mean_and_std[1], mean_and_std[1])

            ret.append((anomaly_type, confidence))

        ret = sorted(ret, key=lambda x: x[1])
        return ret


if __name__ == "__main__":
    from sklearn.externals import joblib
    import glob
    import os
    import pandas as pd

    model_group_by_anomaly_type = {}
    for model in glob.glob('test_data/models/*'):
        with open(model, 'rb') as f:
            model_group_by_anomaly_type[os.path.basename(model)] = joblib.load(f)

    mats = []
    for csv in glob.glob('test_data/csvs/*'):
        with open(csv, 'r') as f:
            df = pd.read_csv(f)
            mats.append(df.values[:, 1:])
    
    c = NormalDistributedConfidenceClassifier(model_group_by_anomaly_type)
    for mat in mats:
        print c.predict(mat)
        print c.predict_proba(mat)
        
