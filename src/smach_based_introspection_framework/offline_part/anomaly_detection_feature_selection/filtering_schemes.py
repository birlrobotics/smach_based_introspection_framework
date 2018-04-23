from geometry_msgs.msg import WrenchStamped
import msg_filters
from rostopics_to_timeseries import RosTopicFilteringScheme

filtering_schemes = []

tfc = RosTopicFilteringScheme()
tfc.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    msg_filters.WrenchStampedFilter(),
)
filtering_schemes.append(tfc)

tfc = RosTopicFilteringScheme()
tfc.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    msg_filters.WrenchStampedNormFilter(),
)
filtering_schemes.append(tfc)
