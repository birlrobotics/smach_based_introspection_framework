from smach_based_introspection_framework._constant import (
    datasets_of_filtering_schemes_folder,
)
import glob
import os
import coloredlogs, logging
import pandas as pd
import ipdb
import re
from filtering_schemes import filtering_schemes


pd.options.display.max_rows = 999

def run():
    logger = logging.getLogger('GenReport')
    logger.setLevel(logging.INFO)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.INFO)
    logger.addHandler(consoleHandler)

    stat_files = glob.glob(os.path.join(
        datasets_of_filtering_schemes_folder,
        'introspection_statistics',
        'No.* filtering scheme',
        'skill *',
        '*.csv',
    ))

    prog = re.compile(r'No.(\d+) filtering scheme%sskill (\d+)'%os.sep)

    big_df = pd.DataFrame()
    for csv in stat_files:
        logger.info(csv)

        m = prog.search(csv)
        if not m:
            raise Exception("Fail to extract scheme no and skill no from %s"%csv)

        scheme_no = m.group(1)
        skill_no = m.group(2)

        df = pd.read_csv(csv) 

        df['scheme_no'] = scheme_no
        df['skill_no'] = skill_no
        df = df.fillna({"anomaly type": "success"}).fillna(0)
    
        big_df = big_df.append(df)
    big_df = big_df.drop(['sample name', big_df.columns[0]], axis=1).set_index(['scheme_no', 'skill_no', 'anomaly type'])
    big_df = big_df.astype(int)

    report = open(os.path.join(datasets_of_filtering_schemes_folder, 'report.txt'), 'w') 
    report.write("scheme info\n")
    report.write("="*30+"\n")
    for scheme_count, filtering_scheme in enumerate(filtering_schemes):
        report.write("scheme %s:\n"%scheme_count)
        report.write("-"*30+"\n")
        report.write(str(filtering_scheme.timeseries_header))
        report.write("\n\n")

    report.write("scheme performance\n")
    report.write("="*30+"\n")
    report.write("granularity: scheme\n")
    report.write("-"*30+"\n")
    report.write(str(big_df.groupby(level=[0]).sum()))
    report.write("\n\n")
    report.write("granularity: skill\n")
    report.write("-"*30+"\n")
    report.write(str(big_df.groupby(level=[0, 1]).sum()))
    report.write("\n\n")
    report.write("granularity: anomaly type\n")
    report.write("-"*30+"\n")
    report.write(str(big_df.groupby(level=[0, 1, 2]).sum()))
    report.write("\n\n")

if __name__ == '__main__':
    run()
