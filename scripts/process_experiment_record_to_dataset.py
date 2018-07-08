#!/usr/bin/env python

import os
import smach_based_introspection_framework.offline_part.process_experiment_record_to_dataset as m1
import ipdb
import logging

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    m1.run()
