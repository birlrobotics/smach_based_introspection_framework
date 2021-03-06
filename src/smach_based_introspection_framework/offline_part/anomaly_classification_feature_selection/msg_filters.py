from rostopics_to_timeseries import TopicMsgFilter
import numpy as np
from collections import deque

class WrenchStampedFilter(TopicMsgFilter):
    def __init__(self):
        super(WrenchStampedFilter, self).__init__()

    def convert(self, msg):
        return [\
            msg.wrench.force.x,\
            msg.wrench.force.y,\
            msg.wrench.force.z,\
            msg.wrench.torque.x,\
            msg.wrench.torque.y,\
            msg.wrench.torque.z,\
        ]

    @staticmethod
    def vector_size():
        return 6

    @staticmethod
    def vector_meaning():
        return ['wrench.force.%s'%i for i in ['x', 'y', 'z']]+\
            ['wrench.torque.%s'%i for i in ['x', 'y', 'z']]

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

    @staticmethod
    def vector_size():
        return 2 

    @staticmethod
    def vector_meaning():
        return [
            'robotiq_force_sensor.wrench.force.norm', \
            'robotiq_force_sensor.wrench.torque.norm',\
        ] 

class WrenchStampedForceNormFilter(TopicMsgFilter):
    def __init__(self):
        super(WrenchStampedForceNormFilter, self).__init__()

    def convert(self, msg):
        force_norm = np.linalg.norm([
            msg.wrench.force.x,\
            msg.wrench.force.y,\
            msg.wrench.force.z,\
        ])
        return [force_norm] 

    @staticmethod
    def vector_size():
        return 1

    @staticmethod
    def vector_meaning():
        return [
            'robotiq_force_sensor.wrench.force.norm', \
        ] 

class WrenchStampedTorqueNormFilter(TopicMsgFilter):
    def __init__(self):
        super(WrenchStampedTorqueNormFilter, self).__init__()

    def convert(self, msg):
        torque_norm = np.linalg.norm([
            msg.wrench.torque.x,\
            msg.wrench.torque.y,\
            msg.wrench.torque.z,\
        ])
        return [torque_norm] 

    @staticmethod
    def vector_size():
        return 1

    @staticmethod
    def vector_meaning():
        return [
            'robotiq_torque_sensor.wrench.torque.norm', \
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

    @staticmethod
    def vector_size():
        return 2 

    @staticmethod
    def vector_meaning():
        return [
            'baxter_enpoint_pose.twist.linear.norm', \
            'baxter_enpoint_pose.twist.angular.norm', \
        ] 

class BaxterEndpointTwistFilter(TopicMsgFilter):
    def __init__(self):
        super(BaxterEndpointTwistFilter, self).__init__()

    def convert(self, msg):
        return [
            msg.twist.linear.x,\
            msg.twist.linear.y,\
            msg.twist.linear.z,\
            msg.twist.angular.x,\
            msg.twist.angular.y,\
            msg.twist.angular.z,\
        ]

    @staticmethod
    def vector_size():
        return 6

    @staticmethod
    def vector_meaning():
        return ['baxter_enpoint_pose.twist.linear.%s'%i for i in ['x', 'y', 'z']]+\
            ['baxter_enpoint_pose.twist.angular.%s'%i for i in ['x', 'y', 'z']]

class HongminTactileFeatureMaxFilter(TopicMsgFilter):
    def __init__(self):
        super(HongminTactileFeatureMaxFilter, self).__init__()

    def convert(self, msg):
        return [msg.tactile_values_4, msg.tactile_values_9] 

    @staticmethod
    def vector_size():
        return 2 

    @staticmethod
    def vector_meaning():
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

    @staticmethod
    def vector_size():
        return 1

    @staticmethod
    def vector_meaning():
        return [
            'tactile_static_data.max', \
        ] 

class TactileStaticMeanFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticMeanFilter, self).__init__()

    def convert(self, msg):
        return [
            np.mean(msg.taxels[0].values),
            np.mean(msg.taxels[1].values),
        ]

    @staticmethod
    def vector_size():
        return 2

    @staticmethod
    def vector_meaning():
        return [
            'tactile_static_data.mean.left', \
            'tactile_static_data.mean.right', \
        ] 

class TactileStaticStdFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticStdFilter, self).__init__()

    def convert(self, msg):
        return [
            np.std(msg.taxels[0].values),
            np.std(msg.taxels[1].values),
        ]

    @staticmethod
    def vector_size():
        return 2

    @staticmethod
    def vector_meaning():
        return [
            'tactile_static_data.left.std', \
            'tactile_static_data.right.std', \
        ] 

class TactileStaticStd1stDerivativeFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticStd1stDerivativeFilter, self).__init__()
        self.prev_f = None

    def convert(self, msg):
        cur_f = [
            np.std(msg.taxels[0].values),
            np.std(msg.taxels[1].values),
        ]
        if self.prev_f is None:
            ret = [0, 0]
        else:
            ret = [cur_f[0]-self.prev_f[0], cur_f[1]-self.prev_f[1]]
        self.prev_f = cur_f
        return ret

    @staticmethod
    def vector_size():
        return 2

    @staticmethod
    def vector_meaning():
        return [
            'tactile_static_data.left.std.1stderivative', \
            'tactile_static_data.right.std.1stderivative', \
        ] 


class TactileDynamicAbsMaxFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileDynamicAbsMaxFilter, self).__init__()

    def convert(self, msg):
        return [max(\
            abs(msg.data[0].value), 
            abs(msg.data[1].value), 
        )]

    @staticmethod
    def vector_size():
        return 1

    @staticmethod
    def vector_meaning():
        return [
            'tactile_dynamic_data.absmax', \
        ] 

class TactileDynamicFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileDynamicFilter, self).__init__()

    def convert(self, msg):
        return [
            msg.data[0].value,
            msg.data[1].value, 
        ]

    @staticmethod
    def vector_size():
        return 2

    @staticmethod
    def vector_meaning():
        return [
            'tactile_dynamic_data.left', \
            'tactile_dynamic_data.right', \
        ] 

class TactileStaticStdEdgeDetectorSize3Filter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticStdEdgeDetectorSize3Filter, self).__init__()
        self.prev_f = deque()

    def convert(self, msg):
        cur_f = [
            np.std(msg.taxels[0].values),
            np.std(msg.taxels[1].values),
        ]
        self.prev_f.append(cur_f)
        if len(self.prev_f) < 3:
            ret = [0, 0]
        else:
            ret = [
                self.prev_f[0][0]+self.prev_f[2][0]-2*self.prev_f[1][0], 
                self.prev_f[0][1]+self.prev_f[2][1]-2*self.prev_f[1][1], 
            ]
            self.prev_f.popleft()
        return ret

    @staticmethod
    def vector_size():
        return 2

    @staticmethod
    def vector_meaning():
        return [
            'tactile_static_data.left.std.EdgeDetectorSize3', \
            'tactile_static_data.right.std.EdgeDetectorSize3', \
        ] 


class TactileStaticStdEdgeDetectorSize5Filter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticStdEdgeDetectorSize5Filter, self).__init__()
        self.prev_f = deque()

    def convert(self, msg):
        cur_f = [
            np.std(msg.taxels[0].values),
            np.std(msg.taxels[1].values),
        ]
        self.prev_f.append(cur_f)
        if len(self.prev_f) < 5:
            ret = [0, 0]
        else:
            ret = [
                -2*self.prev_f[0][0]-self.prev_f[1][0]+self.prev_f[3][0]+2*self.prev_f[4][0], 
                -2*self.prev_f[0][1]-self.prev_f[1][1]+self.prev_f[3][1]+2*self.prev_f[4][1], 
            ]
            self.prev_f.popleft()
        return ret

    @staticmethod
    def vector_size():
        return 2

    @staticmethod
    def vector_meaning():
        return [
            'tactile_static_data.left.std.EdgeDetectorSize5', \
            'tactile_static_data.right.std.EdgeDetectorSize5', \
        ] 
