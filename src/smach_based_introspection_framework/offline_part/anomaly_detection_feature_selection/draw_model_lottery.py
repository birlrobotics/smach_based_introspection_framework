import generate_introspection_models
import collect_detection_statistics
import generate_human_readable_report
import coloredlogs, logging
import datetime
from smach_based_introspection_framework._constant import (
    datasets_of_filtering_schemes_folder,
)
import os
import shutil
import pandas as pd
import ipdb

coloredlogs.install()

folder_time_fmt = "%Yy%mm%dd%HH%MM%SS"

if __name__ == '__main__':
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.INFO)
    logger.addHandler(consoleHandler)

    storage_dir = os.path.join(datasets_of_filtering_schemes_folder, 'lottery_history')
    if not os.path.isdir(storage_dir):
        os.makedirs(storage_dir)

    summary_df = pd.DataFrame() 
    while True:
        postfix = 'start_at_%s'%datetime.datetime.now().strftime(folder_time_fmt)
        generate_introspection_models.run()
        collect_detection_statistics.run()
        big_df = generate_human_readable_report.run()

        
        scheme_level = generate_human_readable_report.append_metrics(big_df.groupby(level=[0]).sum())
        skill_level = generate_human_readable_report.append_metrics(big_df.groupby(level=[0,1]).sum())
        at_level = generate_human_readable_report.append_metrics(big_df.groupby(level=[0,1,2]).sum())

        col_names = ['scheme0 '+i for i in scheme_level.columns]
        values = scheme_level.loc['0'].reshape(-1)
        for idx, c in enumerate(col_names):
            summary_df.loc[postfix, c] = values[idx]

        col_names = ['scheme0_skill5 '+i for i in skill_level.columns]
        values = skill_level.loc[('0', '5')].reshape(-1)
        for idx, c in enumerate(col_names):
            summary_df.loc[postfix, c] = values[idx]

        summary_df.to_csv(os.path.join(datasets_of_filtering_schemes_folder, 'lottery_summary.csv'))

        shutil.move(
            os.path.join(datasets_of_filtering_schemes_folder, 'introspection_models'),
            os.path.join(storage_dir, 'introspection_models.%s'%postfix),
        )

        shutil.move(
            os.path.join(datasets_of_filtering_schemes_folder, 'introspection_statistics'),
            os.path.join(storage_dir, 'introspection_statistics.%s'%postfix),
        )

        shutil.move(
            os.path.join(datasets_of_filtering_schemes_folder, 'report.txt'),
            os.path.join(storage_dir, 'report.txt.%s'%postfix),
        )
