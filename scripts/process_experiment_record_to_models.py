#!/usr/bin/env python

import os
import smach_based_introspection_framework.offline_part.process_experiment_record_to_dataset as m1
import smach_based_introspection_framework.offline_part.process_dataset_to_models as m2

if __name__ == "__main__":
    m1.run()
    m2.run()
