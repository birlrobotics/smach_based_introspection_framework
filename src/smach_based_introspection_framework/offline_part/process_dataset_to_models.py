import glob
import os,ipdb
from smach_based_introspection_framework._constant import (
    latest_dataset_folder,
    latest_model_folder,
)
import re
import logging
import gen_introspection_model, gen_classification_model, gen_dmp_model


def get_latest_model_folder():
    if not os.path.isdir(latest_model_folder):
        os.makedirs(latest_model_folder)
    return latest_model_folder

def run():
    if not os.path.isdir(latest_dataset_folder):
        raise Exception("Not found %s"%latest_dataset_folder)

    logger = logging.getLogger('process_dataset_to_models')
    logger.setLevel(logging.DEBUG)
    log_file = os.path.join(get_latest_model_folder(), 'run.log')
    fileHandler = logging.FileHandler(os.path.realpath(log_file))
    fileHandler.setLevel(logging.DEBUG)
    logger.addHandler(fileHandler)
    '''
    skill_folders = glob.glob(os.path.join(
        latest_dataset_folder,
        'skill_data',
        "tag_*",
    ))
    prog = re.compile(r'tag_(.*)')
    for skill_folder in skill_folders:
        skill_id = prog.match(os.path.basename(skill_folder)).group(1)
        logger.debug("Processing skill %s"%skill_id)
        output_dir = os.path.join(
            get_latest_model_folder(), 
            'skill_%s'%skill_id,
        )
        gen_introspection_model.run(logger, skill_folder, output_dir)

    anomaly_folders = glob.glob(os.path.join(
        latest_dataset_folder,
        'anomaly_data',
        "anomaly_type_*",
    ))
    prog = re.compile(r'anomaly_type_(.*)')
    for anomaly_folder in anomaly_folders:
        m = prog.search(os.path.basename(anomaly_folder))
        anomaly_label = m.group(1)
        logger.debug("Processing anomaly %s"%(anomaly_label, ))

        output_dir = os.path.join(
            get_latest_model_folder(), 
            'anomaly_%s'%anomaly_label,
        )
        gen_classification_model.run(logger, anomaly_folder, output_dir)
    '''

    demonstration_folders = glob.glob(os.path.join(
        latest_dataset_folder,
        'demonstration_data',
        "nominal_skill_*_anomaly_type_*_tag_*",
    ))
    prog = re.compile(r'nominal_skill_(.*)_anomaly_type_(.*)_tag_(.*)')
    for demonstration_folder in demonstration_folders:
        m = prog.match(os.path.basename(demonstration_folder))
        skill_id = m.group(1)
        anomaly_label = m.group(2)
        demonstration_skill_id = m.group(3)
        logger.debug("Processing skill %s's anomaly %s's demonstration skill %s"%(skill_id, anomaly_label, demonstration_skill_id))
        # TODO: train dmp models

        output_dir = os.path.join(
            get_latest_model_folder(), 
            'skill_%s'%demonstration_skill_id,
        )
        gen_dmp_model.run(logger, demonstration_folder, output_dir)

if __name__=="__main__":
    run()
