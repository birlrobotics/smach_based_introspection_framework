from smach_based_introspection_framework._constant import (
    anomaly_classification_feature_selection_folder,
)
from smach_based_introspection_framework.configurables import (
    model_config, train_size
)
import glob
import os
import coloredlogs, logging
import pandas as pd
import ipdb
import re
from ac_feature_schemes import feature_schemes

pd.set_option("display.max_rows", None)
pd.set_option("display.max_columns", None)
#pd.set_option("display.width", 1000)
#pd.set_option("display.height", 1000)
pd.set_option("display.precision", 3)

def append_metrics(df_):
    try:
        df = df_.copy()
        for k in ['TP', 'TN', 'FP', 'FN']:
            if k not in df.columns:
                df[k] = 0

	
        df['precision'] = df['TP']/(df['TP']+df['FP']) 
        df['recall'] = df['TP']/(df['TP']+df['FN']) 
        df['F1score'] = 2*df['TP']/(2*df['TP']+df['FP']+df['FN'])

        df['accuracy'] = (df['TP']+df['TN'])/(df['TP']+df['TN']+df['FP']+df['FN']) 
    except Exception as e:
        raise e
    return df

def run():
    logger = logging.getLogger('GenReport')

    stat_files = glob.glob(os.path.join(
        anomaly_classification_feature_selection_folder,
        'classification_statistics',
        'No.* filtering scheme',
        '*.csv',
    ))

    prog = re.compile(r'No.(\d+) filtering scheme')

    big_df = pd.DataFrame()
    for csv in stat_files:
        logger.info(csv)

        m = prog.search(csv)
        if not m:
            raise Exception("Fail to extract scheme no from %s"%csv)

        scheme_no = m.group(1)

        df = pd.read_csv(csv) 
        df['scheme_no'] = scheme_no
    
        big_df = big_df.append(df)

    proba_cols = [i for i in big_df.columns if i.startswith("proba_of")]
    big_df = big_df.drop(['Unnamed: 0', 'anomaly_type_given_by_human', 'anomaly_csv']+proba_cols, axis=1).set_index(['scheme_no', 'anomaly_type_being_tested'])
    big_df = big_df.fillna(0).astype(int)


    config = ''.join('{}{}'.format(key, val) for key, val in sorted(model_config.items()))
    config = config + ('[{}]'.format(train_size))
    report = open(os.path.join(anomaly_classification_feature_selection_folder, config+'-report.txt'), 'w') 
    report.write("scheme info\n")
    report.write("="*30+"\n")
    for scheme_count, feature_scheme in enumerate(feature_schemes):
        report.write("scheme %s:\n"%scheme_count)
        report.write("-"*30+"\n")
        report.write(feature_scheme.one_line_info)
        report.write("\n\n")

    report.write("scheme performance\n")
    report.write("="*30+"\n")
    report.write("granularity: scheme\n")
    report.write("-"*30+"\n")
    report.write(str(append_metrics(big_df.groupby(level=[0]).sum())))
    report.write("\n\n")
    report.write("granularity: anomaly type\n")
    report.write("-"*30+"\n")
    report.write(str(append_metrics(big_df.groupby(level=[0, 1]).sum())))
    report.write("\n\n")
    logger.info('Finish running: ' + os.path.basename(__file__))
    logger.info('report.txt Done')
    return big_df

if __name__ == '__main__':
    run()
