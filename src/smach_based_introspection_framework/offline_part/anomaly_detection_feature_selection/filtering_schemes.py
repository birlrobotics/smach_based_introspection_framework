from geometry_msgs.msg import WrenchStamped
from msg_filters import WrenchStampedFilter
from rostopics_to_timeseries import RosTopicFilteringScheme

filtering_schemes = []

tfc = RosTopicFilteringScheme()
tfc.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    WrenchStampedFilter(),
)
filtering_schemes.append(tfc)
