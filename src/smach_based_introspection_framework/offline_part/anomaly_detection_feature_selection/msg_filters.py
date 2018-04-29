from rostopics_to_timeseries import TopicMsgFilter
import numpy as np

class WrenchStampedFilter(TopicMsgFilter):
    def __init__(self):
        super(WrenchStampedFilter, self).__init__()

    def convert(self, msg):
        return [\
            msg.wrench.force.x,\
            msg.wrench.force.y,\
            msg.wrench.force.z,\
        ]

    def vector_size(self):
        return 3

    def vector_meaning(self):
        return ['wrench.force.%s'%i for i in ['x', 'y', 'z']] 

class WrenchStampedNormFilter(TopicMsgFilter):
    def __init__(self):
        super(WrenchStampedNormFilter, self).__init__()

    def convert(self, msg):
        force_norm = np.linalg.norm([
            msg.wrench.force.x,\
            msg.wrench.force.y,\
            msg.wrench.force.z,\
        ])
        torque_norm = np.linalg.norm([
            msg.wrench.torque.x,\
            msg.wrench.torque.y,\
            msg.wrench.torque.z,\
        ])
        return [force_norm, torque_norm] 
    def vector_size(self):
        return 2 

    def vector_meaning(self):
        return [
            'robotiq_force_sensor.wrench.force.norm', \
            'robotiq_force_sensor.wrench.torque.norm',\
        ] 

class BaxterEndpointTwistNormFilter(TopicMsgFilter):
    def __init__(self):
        super(BaxterEndpointTwistNormFilter, self).__init__()

    def convert(self, msg):
        linear_norm = np.linalg.norm([
            msg.twist.linear.x,\
            msg.twist.linear.y,\
            msg.twist.linear.z,\
        ])
        angular_norm = np.linalg.norm([
            msg.twist.angular.x,\
            msg.twist.angular.y,\
            msg.twist.angular.z,\
        ])
        return [linear_norm, angular_norm] 
    def vector_size(self):
        return 2 

    def vector_meaning(self):
        return [
            'baxter_enpoint_pose.twist.linear.norm', \
            'baxter_enpoint_pose.twist.angular.norm', \
        ] 

class HongminTactileFeatureMaxFilter(TopicMsgFilter):
    def __init__(self):
        super(HongminTactileFeatureMaxFilter, self).__init__()

    def convert(self, msg):
        return [msg.tactile_values_4, msg.tactile_values_9] 

    def vector_size(self):
        return 2 

    def vector_meaning(self):
        return [
            'left_tactile_sensor.max', \
            'right_tactile_sensor.max', \
        ] 

class TactileStaticMaxFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticMaxFilter, self).__init__()

    def convert(self, msg):
        return [max(\
            max(msg.taxels[0].values),
            max(msg.taxels[1].values),
        )]

    def vector_size(self):
        return 1

    def vector_meaning(self):
        return [
            'tactile_static_data.max', \
        ] 

class TactileStaticMeanFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticMeanFilter, self).__init__()

    def convert(self, msg):
        return [np.mean([\
            np.mean(msg.taxels[0].values),
            np.mean(msg.taxels[1].values),
        ])]

    def vector_size(self):
        return 1

    def vector_meaning(self):
        return [
            'tactile_static_data.mean', \
        ] 

class TactileStaticStdFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticStdFilter, self).__init__()

    def convert(self, msg):
        return [
            np.std(msg.taxels[0].values),
            np.std(msg.taxels[1].values),
        ]

    def vector_size(self):
        return 2

    def vector_meaning(self):
        return [
            'tactile_static_data.left.std', \
            'tactile_static_data.right.std', \
        ] 

class TactileDynamicAbsMaxFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileDynamicAbsMaxFilter, self).__init__()

    def convert(self, msg):
        return [max(\
            abs(msg.data[0].value), 
            abs(msg.data[1].value), 
        )]

    def vector_size(self):
        return 1

    def vector_meaning(self):
        return [
            'tactile_dynamic_data.absmax', \
        ] 
