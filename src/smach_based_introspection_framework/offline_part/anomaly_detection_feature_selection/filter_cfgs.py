from geometry_msgs.msg import WrenchStamped
from msg_filters import WrenchStampedFilter
from rostopics_to_timeseries import RosTopicFilteringConfig

filter_cfgs = []

tfc = RosTopicFilteringConfig()
tfc.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    WrenchStampedFilter(),
)
filter_cfgs.append(tfc)
