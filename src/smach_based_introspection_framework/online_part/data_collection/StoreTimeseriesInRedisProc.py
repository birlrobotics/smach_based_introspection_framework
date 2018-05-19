
import multiprocessing
from rostopics_to_timeseries.msg import (
    Timeseries,
)
import redis
import rospy

class StoreTimeseriesInRedisProc(multiprocessing.Process):
    def __init__(
        self, 
        node_name="StoreTimeseriesInRedisProc_node",
        topic_name="/rostopics_to_timeseries_topic",
    ):
        super(StoreTimeseriesInRedisProc, self).__init__()

        self.node_name = node_name
        self.topic_name = topic_name

    def callback_timeseries(self, msg):
        value = msg.sample
        score = msg.header.stamp.to_sec()
        self.redis_handle.zadd(self.topic_name, value, score)

    def _setup_redis(self):
        r = redis.Redis(host='localhost', port=6379, db=0)
        rospy.loginfo('delete key \"%s\": %s'%(self.topic_name, r.delete(self.topic_name)))
        self.redis_handle = r

    def run(self):
        rospy.init_node(self.node_name, anonymous=True)

        self._setup_redis()

        rospy.Subscriber(self.topic_name, Timeseries, self.callback_timeseries)
        rospy.spin()

if __name__ == '__main__':
    proc = StoreTimeseriesInRedisProc()
    proc.daemon = True

    proc.start()
    proc.join()
