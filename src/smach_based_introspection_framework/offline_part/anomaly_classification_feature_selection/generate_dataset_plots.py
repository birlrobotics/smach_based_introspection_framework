import itertools
from smach_based_introspection_framework._constant import (
    anomaly_classification_feature_selection_folder,
)
import glob
import os
import pandas as pd
import coloredlogs, logging
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pickle
import re
import ipdb

def run():
    logger = logging.getLogger('GenPlots')

    folders = glob.glob(os.path.join(
        anomaly_classification_feature_selection_folder,
        'No.* filtering scheme',
        'anomalies_grouped_by_type',
        'anomaly_type_(*)',
    )) 

    logger.info(folders)

    at_extractor = re.compile(r'anomaly_type_\((.*)\)')
    
    for folder in itertools.chain(folders):
        logger.info(folder)
        path_postfix = os.path.relpath(folder, anomaly_classification_feature_selection_folder)

        anomaly_type = at_extractor.search(path_postfix).group(1)

        output_dir = os.path.join(anomaly_classification_feature_selection_folder, "visualized dataset", path_postfix)
        csv_files = glob.glob(os.path.join(folder, '*', '*.csv'))
        csvs_by_type = len(csv_files)
        fig_whole_type, axarr = plt.subplots(nrows=csvs_by_type, ncols=1, figsize=(24, 8*csvs_by_type))
        flag = None
        for num, csv in enumerate(glob.glob(os.path.join(folder, '*', '*.csv'))):
    
            logger.info(csv)
            exp_name = os.path.basename(csv)

            anomaly_t_to_plot = []
            with open(os.path.join(os.path.dirname(csv), "anomaly_time.pkl"), 'rb') as f:
                anomaly_t = pickle.load(f).to_sec()

            l2_output_dir = os.path.join(output_dir, exp_name)
            if not os.path.isdir(l2_output_dir):
                os.makedirs(l2_output_dir)

            df = pd.read_csv(csv)
            if flag is None:
                flag = 'Enter'
                ndims = len(df.columns)-1
                fig_dims, axs = plt.subplots(nrows=ndims, ncols=1, figsize=(24, 6*ndims))
            for idx, col_name in enumerate(df.columns):
                fig, ax = plt.subplots(nrows=1, ncols=1)
                if idx == 0:
                    xs = df.iloc[:, 0]
                    
                    axax = axarr[num] if csvs_by_type > 1 else axarr 
                    df.plot(ax=axax, x=df.keys()[0], y=df.keys()[1::], title=exp_name, colormap='jet', legend=True)
                    axax.legend(loc=2, framealpha=0.3, ncol=1)                        
                    axax.axvline(anomaly_t, c='purple')
                    ymin,ymax = axax.get_ylim()
                    axax.text(anomaly_t, ymax-0.25*(ymax-ymin), '%s'%anomaly_type, color='purple', rotation=-90)
                    continue
                else:
                    ys = df.iloc[:, idx]
                    ax.plot(xs, ys)
                    if num == 0:
                        xxs = xs
                        axs[idx-1].plot(xxs, ys,label=col_name)
                        axs[idx-1].legend(loc=2, fancybox=True, framealpha=0.3)
                    elif num == csvs_by_type - 1:
                        axs[idx-1].axvline(xxs.median(), c='purple')
                        ymin,ymax = axs[idx-1].get_ylim()
                        axs[idx-1].text(xxs.median(), ymax-0.25*(ymax-ymin), '%s'%anomaly_type, color='purple', rotation=-90)
                    else:
                        axs[idx-1].plot(xxs, ys)

                output_f = os.path.join(l2_output_dir, "dim %s.png"%col_name.replace('/', 'divide'))
                if os.path.exists(output_f):
                    continue
                xmin,xmax = ax.get_xlim()
                ymin,ymax = ax.get_ylim()
                ax.axvline(anomaly_t, c='purple')
                ax.text(anomaly_t, ymax-0.25*(ymax-ymin), '%s'%anomaly_type, color='purple', rotation=-90)                    
                ax.set_title("[%s] [%s] [%s]"%(anomaly_type, exp_name, col_name))
                fig.set_size_inches(16, 4)
                with open(output_f, 'w') as f:
                    fig.savefig(f, dpi=300)
                plt.close(fig)
        save_path = os.path.join(output_dir, anomaly_type + '.pdf')
        fig_whole_type.savefig(save_path, dpi=300)
        dims_save_path = os.path.join(output_dir, anomaly_type + '_dims.pdf')
        fig_dims.savefig(dims_save_path, dpi=300)
        

if __name__ == '__main__':
    run()
    
