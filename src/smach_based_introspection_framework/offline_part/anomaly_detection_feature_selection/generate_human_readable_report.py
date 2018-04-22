from smach_based_introspection_framework._constant import (
    datasets_of_filtering_schemes_folder,
)
import glob
import os
import coloredlogs, logging
import pandas as pd
import ipdb
import re

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
    
        big_df = big_df.append(df)
    ipdb.set_trace()
    #TODO sumarize and generate human readiable txt report.

if __name__ == '__main__':
    run()
