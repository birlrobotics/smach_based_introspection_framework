import itertools
import ac_variables
from filtering_schemes import filtering_schemes

class AnomalyClassificationFeatureScheme(object):
    def __init__(self, window_size_config_arg, filtering_scheme):
        self.window_size_config_arg = window_size_config_arg
        self.filtering_scheme = filtering_scheme

    @property
    def detailed_info(self):
        info = ""
        info += "window_size_config_arg: %s\n"%dict(zip(['secs_before', 'secs_after'], self.window_size_config_arg))
        info += self.filtering_scheme.info
        return info

    @property
    def one_line_info(self):
        info = "window_size_config_arg: %s, filtering_headers: %s"%(dict(zip(['secs_before', 'secs_after'], self.window_size_config_arg)), self.filtering_scheme.timeseries_header)
        return info
        
feature_schemes = [AnomalyClassificationFeatureScheme(i[0], i[1]) for i in itertools.product(ac_variables.window_size_config_args, filtering_schemes)]

if __name__ == '__main__':
    import coloredlogs, logging
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    for fs in feature_schemes:
        logger.info(fs.one_line_info)
        logger.info(fs.detailed_info)
