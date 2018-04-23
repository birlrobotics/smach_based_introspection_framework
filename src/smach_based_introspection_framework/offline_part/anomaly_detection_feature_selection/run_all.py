import generate_succ_and_unsucc_datasets
import generate_dataset_plots
import generate_introspection_models
import collect_detection_statistics
import generate_human_readable_report
import coloredlogs, logging
coloredlogs.install()

if __name__ == '__main__':
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.DEBUG)
    logger.addHandler(consoleHandler)
    generate_succ_and_unsucc_datasets.run()
    generate_dataset_plots.run()
    generate_introspection_models.run()
    collect_detection_statistics.run()
    generate_human_readable_report.run()
