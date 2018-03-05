from geometry_msgs.msg import Wrench
import ipdb

class Bunch:
    def __init__(self, **kwds):
        self.__dict__.update(kwds)

class TagTopicMsgDecorator(object):
    def __init__(self):
        self.last_msg = None

    def decorate(self, msg):
        delta_wrench = Wrench()
        if self.last_msg is not None:
            t0 = self.last_msg.wrench_stamped.wrench
            t1 = msg.wrench_stamped.wrench
            delta_wrench.force.x = t1.force.x-t0.force.x
            delta_wrench.force.y = t1.force.y-t0.force.y
            delta_wrench.force.z = t1.force.z-t0.force.z
            delta_wrench.torque.x = t1.torque.x-t0.torque.x
            delta_wrench.torque.y = t1.torque.y-t0.torque.y
            delta_wrench.torque.z = t1.torque.z-t0.torque.z
        else:
            delta_wrench.force.x = 0
            delta_wrench.force.y = 0
            delta_wrench.force.z = 0
            delta_wrench.torque.x = 0
            delta_wrench.torque.y = 0
            delta_wrench.torque.z = 0
        
        bunch = Bunch()
        for attr in msg.__slots__:
            setattr(bunch, attr, getattr(msg, attr)) 
        bunch.delta_wrench = delta_wrench    

        self.last_msg = msg

        return bunch
