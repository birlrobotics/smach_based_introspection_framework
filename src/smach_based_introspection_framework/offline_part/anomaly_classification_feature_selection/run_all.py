import generate_anomaly_datasets
import generate_classifer_models
import collect_classification_statistics
import generate_human_readable_report
import generate_dataset_plots

if __name__ == '__main__':
    generate_anomaly_datasets.run()
    generate_classifer_models.run()
    collect_classification_statistics.run()
    generate_human_readable_report.run()
    generate_dataset_plots.run()
