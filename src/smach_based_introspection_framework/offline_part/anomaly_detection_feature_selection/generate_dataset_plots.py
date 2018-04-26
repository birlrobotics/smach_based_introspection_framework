
import itertools
from smach_based_introspection_framework._constant import (
    datasets_of_filtering_schemes_folder,
)
import glob
import os
import pandas as pd
import coloredlogs, logging
import matplotlib.pyplot as plt

def run():
    logger = logging.getLogger('GenPlots')
    logger.setLevel(logging.INFO)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.INFO)
    logger.addHandler(consoleHandler)

    succ_folders = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'No.* filtering scheme',
        'successful_skills',
        'skill *',
    )) 

    unsucc_folders = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'No.* filtering scheme',
        'unsuccessful_skills',
        'skill *',
    )) 

    whole_exp_folders = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'No.* filtering scheme',
        'whole_experiment',
    )) 

    for folder in itertools.chain(succ_folders, unsucc_folders, whole_exp_folders):
        logger.info(folder)
        path_postfix = os.path.relpath(folder, datasets_of_filtering_schemes_folder)

        output_dir = os.path.join(datasets_of_filtering_schemes_folder, "visualized dataset", path_postfix)

        xs = None
        args = {}
        for csv in glob.glob(os.path.join(folder, '*', '*.csv')):
            logger.info(csv)
            df = pd.read_csv(csv)
            for idx, name in enumerate(df.columns):
                if idx == 0:
                    xs = df.iloc[:, 0]
                    xs = xs-xs[0]
                else:
                    if name not in args:
                        args[name] = []
                    args[name].append(xs)
                    args[name].append(df.iloc[:, idx])

        for key, value in args.iteritems():
            fig, ax = plt.subplots(nrows=1, ncols=1)
            ax.plot(*value)
            ax.set_title(path_postfix+'\n'+key)
    
            fig_file = os.path.join(output_dir, "featuer %s.png"%key)

            if not os.path.isdir(output_dir):
                os.makedirs(output_dir)
            fig.set_size_inches(64, 16)
            fig.savefig(fig_file)

if __name__ == '__main__':
    run()
    
