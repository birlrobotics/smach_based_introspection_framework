
import itertools
from smach_based_introspection_framework._constant import (
    datasets_of_filtering_schemes_folder,
)
import glob
import os
import pandas as pd
import coloredlogs, logging
import matplotlib.pyplot as plt
import pickle

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


    for folder in itertools.chain(whole_exp_folders):
        logger.info(folder)
        path_postfix = os.path.relpath(folder, datasets_of_filtering_schemes_folder)
        output_dir = os.path.join(datasets_of_filtering_schemes_folder, "visualized dataset", path_postfix)
        for csv in glob.glob(os.path.join(folder, '*', '*.csv')):
    
            logger.info(csv)
            exp_name = os.path.basename(csv)

            with open(os.path.join(os.path.dirname(csv), "tag_ranges.pkl"), 'rb') as f:
                tag_ranges = pickle.load(f) 
                tag_starts_to_plot = []
                for tag, (st, et) in tag_ranges:
                    tag_starts_to_plot.append((tag, st.to_sec(), et.to_sec()))

            anomaly_t_to_plot = []
            try:
                with open(os.path.join(os.path.dirname(csv), "anomaly_signals.pkl"), 'rb') as f:
                    anomaly_signals = pickle.load(f) 
                    for atype, astamp in anomaly_signals:
                        anomaly_t_to_plot.append((atype, astamp.to_sec()))
            except:
                pass



            l2_output_dir = os.path.join(output_dir, exp_name)
            if not os.path.isdir(l2_output_dir):
                os.makedirs(l2_output_dir)

            df = pd.read_csv(csv)
            for idx, col_name in enumerate(df.columns):
                if idx == 0:
                    xs = df.iloc[:, 0]
                else:
                    fig, ax = plt.subplots(nrows=1, ncols=1)
                    ys = df.iloc[:, idx]
                    ax.plot(xs, ys)
                    xmin,xmax = ax.get_xlim()
                    ymin,ymax = ax.get_ylim()
                    for tag, start, end in tag_starts_to_plot:
                        if int(tag) != 0:
                            if int(tag) < 0:
                                c = 'red'
                            else:
                                c = 'blue'
                            ax.axvline(start, c=c)
                            ax.text(start, ymax-0.05*(ymax-ymin), 'tag %s'%tag, color=c, rotation=-90)
                        else:
                            ax.axvline(start, c='gray')

                    for atype, at in anomaly_t_to_plot:
                        ax.axvline(at, c='purple')
                        ax.text(at, ymax-0.25*(ymax-ymin), '%s'%atype, color='purple', rotation=-90)

                    ax.set_title("%s %s"%(exp_name, col_name))
                    output_f = os.path.join(l2_output_dir, "dim %s.png"%col_name)
                    fig.set_size_inches(64, 16)
                    with open(output_f, 'w') as f:
                        fig.savefig(f)
                    plt.close(fig)


    for folder in itertools.chain(succ_folders, unsucc_folders):
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
            with open(fig_file, 'w') as f:
                fig.savefig(f)
            plt.close(fig)

if __name__ == '__main__':
    run()
    
