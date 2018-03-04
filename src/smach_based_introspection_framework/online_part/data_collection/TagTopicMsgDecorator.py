from geometry_msgs.msg import Wrench
import ipdb
from baxter_interface import Limb

class Bunch:
    def __init__(self, **kwds):
        self.__dict__.update(kwds)

class TagTopicMsgDecorator(object):
    def __init__(self):
        self.last_msg = None
        self.limb = Limb('right')

    def decorate(self, msg):
        delta_wrench = Wrench()
        if self.last_msg is not None:
            t0 = self.last_msg.wrench_stamped.wrench
            t1 = msg.wrench_stamped.wrench
            delta_wrench.force.x = t1.force.x-t0.force.x
            delta_wrench.force.y = t1.force.y-t0.force.y
            delta_wrench.force.z = t1.force.z-t0.force.z
        else:
            delta_wrench.force.x = 0
            delta_wrench.force.y = 0
            delta_wrench.force.z = 0
        
        bunch = Bunch()
        for attr in msg.__slots__:
            setattr(bunch, attr, getattr(msg, attr)) 
        bunch.delta_wrench = delta_wrench    

        ja = self.limb.joint_angles
        bunch.joint_angles = Bunch()
        bunch.joint_angles.right_s0 = ja['right_s0']
        bunch.joint_angles.right_s1 = ja['right_s1']
        bunch.joint_angles.right_e0 = ja['right_e0']
        bunch.joint_angles.right_e1 = ja['right_e1']
        bunch.joint_angles.right_w0 = ja['right_w0']
        bunch.joint_angles.right_w1 = ja['right_w1']
        bunch.joint_angles.right_w2 = ja['right_w2']

        self.last_msg = msg

        return bunch
