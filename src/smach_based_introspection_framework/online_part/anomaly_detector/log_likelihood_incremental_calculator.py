import numpy as np
from scipy.misc import logsumexp

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

class BasicCalculator(object):
    def __init__(self, model):
        self.model = model
        self.samples = []

    def add_one_sample_and_get_loglik(self, sample):
        self.samples.append(sample)
        return self.model.score(np.concatenate(self.samples, axis=0))

def get_calculator(model):
    import hmmlearn.hmm
    if issubclass(type(model), hmmlearn.hmm._BaseHMM):
        return HmmlearnModelIncrementalLoglikCalculator(model)
        
    else:
        print "Returning BasicCalculator! HMM incremental calculation will NOT be optimal."
        return BasicCalculator(model)
