import itertools
from smach_based_introspection_framework._constant import (
    anomaly_classification_feature_selection_folder,
)
import glob
import os
import pandas as pd
import coloredlogs, logging
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
        num_by_type = len(csv_files)
        fig_whole_type, axarr = plt.subplots(nrows=num_by_type, ncols=1)        
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
            for idx, col_name in enumerate(df.columns):
                fig, ax = plt.subplots(nrows=1, ncols=1)
                if idx == 0:
                    xs = df.iloc[:, 0]
                    df.plot(ax=ax, x=df.keys()[0],y=df.keys()[1::])
                    ax.legend(loc=2, bbox_to_anchor=(1.05, 1), fancybox=True, shadow=True, ncol=1)
                    if num_by_type <= 1:
                        df.plot(ax=axarr, x=df.keys()[0], y=df.keys()[1::], title=exp_name , legend=False)
                        axarr.axvline(anomaly_t, c='purple')
                        ymin,ymax = axarr.get_ylim()
                        axarr.text(anomaly_t, ymax-0.25*(ymax-ymin), '%s'%anomaly_type, color='purple', rotation=-90)                           
                    else:
                        df.plot(ax=axarr[num], x=df.keys()[0], y=df.keys()[1::], title=exp_name, legend=False)
                        axarr[num].axvline(anomaly_t, c='purple')
                        ymin,ymax = axarr[num].get_ylim()
                        axarr[num].text(anomaly_t, ymax-0.25*(ymax-ymin), '%s'%anomaly_type, color='purple', rotation=-90)   
                else:
                    ys = df.iloc[:, idx]
                    ax.plot(xs, ys)                    
                xmin,xmax = ax.get_xlim()
                ymin,ymax = ax.get_ylim()

                ax.axvline(anomaly_t, c='purple')
                ax.text(anomaly_t, ymax-0.25*(ymax-ymin), '%s'%anomaly_type, color='purple', rotation=-90)                    
                ax.set_title("[%s] [%s] [%s]"%(anomaly_type, exp_name, col_name))
                output_f = os.path.join(l2_output_dir, "dim %s.png"%col_name.replace('/', 'divide'))
                fig.set_size_inches(16, 4)
                with open(output_f, 'w') as f:
                    fig.savefig(f)
                plt.close(fig)
        save_path = os.path.join(output_dir, anomaly_type + '.png')
        fig_whole_type.savefig(save_path, dpi=300)
        

if __name__ == '__main__':
    run()
    
