import numpy as np

from scipy.misc import logsumexp
import ipdb

def log_mask_zero(a):
    """Computes the log of input probabilities masking divide by zero in log.
    Notes
    -----
    During the M-step of EM-algorithm, very small intermediate start
    or transition probabilities could be normalized to zero, causing a
    *RuntimeWarning: divide by zero encountered in log*.
    This function masks this unharmful warning.
    """
    a = np.asarray(a)
    with np.errstate(divide="ignore"):
        a_log = np.log(a)
        a_log[a <= 0] = 0.0
        return a_log

class HmmlearnModelIncrementalLoglikCalculator(object):
    def __init__(self, model):
        self.model = model
        self.n_components = model.n_components
        self.log_transmat = log_mask_zero(model.transmat_) 
        self.log_startprob = log_mask_zero(model.startprob_)
        self.fwdlattice = None
        self.work_buffer = np.zeros(self.n_components)

    def add_one_sample_and_get_loglik(self, sample):
        framelogprob = self.model._compute_log_likelihood(sample) 
        if self.fwdlattice is None:
            self.fwdlattice = np.zeros((1, self.n_components))
            for i in range(self.n_components):
                self.fwdlattice[0, i] = self.log_startprob[i] + framelogprob[0, i]
        else:
            self.fwdlattice = np.append(self.fwdlattice, np.zeros((1, self.n_components)), axis=0)
            for j in range(self.n_components):
                for i in range(self.n_components):
                    self.work_buffer[i] = self.fwdlattice[-2, i] + self.log_transmat[i, j]

                self.fwdlattice[-1, j] = logsumexp(self.work_buffer) + framelogprob[0, j]
        
        return logsumexp(self.fwdlattice[-1])

class BNPYModelIncrementalLoglikCalculator(object):
    def __init__(self, model):
        model = model.model
        self.model = model
        self.n_components = model.allocModel.K
        self.log_startprob = np.log(model.allocModel.get_init_prob_vector()) 
        self.log_transmat  = np.log(model.allocModel.get_trans_prob_matrix())
        self.fwdlattice = None
        self.preSample = None
        self.work_buffer = np.zeros(self.n_components)

    def add_one_sample_and_get_loglik(self, sample):
        import bnpy
        if self.preSample is None:
            self.preSample = sample
            Xprev  = np.array([sample])
            X      = np.array([sample])            
        else:
            Xprev = self.preSample
            X = sample
            self.preSample = sample
        length = 1
        doc_range = [0, length]
        dataset = bnpy.data.GroupXData(X, doc_range, length, Xprev)

        '''
        LP = self.model.calc_local_params(dataset)
        framelogprob = LP['E_log_soft_ev'] 
        if self.fwdlattice is None:
            self.fwdlattice = np.zeros((1, self.n_components))
            for i in range(self.n_components):
                self.fwdlattice[0,i] = self.log_startprob[i] + framelogprob[0,i]
        else:
            self.fwdlattice = np.append(self.fwdlattice, np.zeros((1, self.n_components)), axis=0)
            for j in range(self.n_components):
                for i in range(self.n_components):
                    self.work_buffer[i] = self.fwdlattice[-2,i] + self.log_transmat[i,j]
                self.fwdlattice[-1,j] = logsumexp(self.work_buffer) + framelogprob[0,j]
        curr_log = logsumexp(self.fwdlattice[-1])
        '''
        
        logSoftEv = self.model.obsModel.calcLogSoftEvMatrix_FromPost(dataset)
        SoftEv, lognormC = bnpy.allocmodel.hmm.HMMUtil.expLogLik(logSoftEv)
        PiMat = np.exp(self.log_transmat)
        if self.fwdlattice is None:
            self.fwdlattice = np.exp(self.log_startprob) * SoftEv 
        else:
            self.fwdlattice = np.dot(PiMat.T, self.fwdlattice[0]) * SoftEv
        margPrObs = np.sum(self.fwdlattice)
        self.fwdlattice /= margPrObs
        curr_log = np.log(margPrObs) + lognormC        
        return curr_log

class BasicCalculator(object):
    def __init__(self, model):
        self.model = model
        self.samples = []

    def add_one_sample_and_get_loglik(self, sample):
        self.samples.append(sample)
        return self.model.score(np.concatenate(self.samples, axis=0))


def get_calculator(model):
    import hmmlearn.hmm
    from birl_hmm.bnpy_hmm_wrapper.hmm import HongminHMM
    if issubclass(type(model), hmmlearn.hmm._BaseHMM):
        return HmmlearnModelIncrementalLoglikCalculator(model)
    elif issubclass(type(model), HongminHMM):
        return BNPYModelIncrementalLoglikCalculator(model)
    else:
        print "Returning BasicCalculator! HMM incremental calculation will NOT be optimal."
        return BasicCalculator(model)


if __name__ == '__main__':
    from sklearn.externals import joblib
    from birl_hmm.hmm_training.hmm_util import fast_log_curve_calculation
    import matplotlib.pyplot as plt
    import numpy
    model = joblib.load('test_data/introspection_model')['hmm_model']
    mat = numpy.load('test_data/test_mat.npy')
    c = get_calculator(model) 
    baseline_curve = fast_log_curve_calculation(mat, model)

    test_curve = []
    for i in range(mat.shape[0]):
        test_curve.append(c.add_one_sample_and_get_loglik(mat[i].reshape((1, -1))))

    fig, ax = plt.subplots(nrows=1, ncols=1)
    ax.plot(baseline_curve, '-g') 
    ax.plot(test_curve, '-r') 

    plt.show()








