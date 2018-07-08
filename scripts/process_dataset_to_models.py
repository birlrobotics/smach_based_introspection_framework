#!/usr/bin/env python

import os
import smach_based_introspection_framework.offline_part.process_dataset_to_models as m2
import ipdb
import logging

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    m2.run()
