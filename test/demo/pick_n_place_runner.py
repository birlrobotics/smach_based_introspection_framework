#!/usr/bin/env python
import baxter_interface

import rospy

import smach_ros

from smach_based_introspection_framework.online_part import (
    smach_runner
)


def shutdown():
    pass

if __name__ == '__main__':
    from task_states import assembly_user_defined_sm

    sm = assembly_user_defined_sm()

    smach_runner.run(sm)
