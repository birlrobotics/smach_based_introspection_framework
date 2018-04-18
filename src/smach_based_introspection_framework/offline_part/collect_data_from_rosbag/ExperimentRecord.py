import os
import glob
import rosbag

class ExperimentRecord(object):
    def __init__(self, folder_path):
        self.folder_path = folder_path

    @property
    def tag_ranges(self):
        if hasattr(self, "_tag_ranges"):
            return self._tag_ranges

        last_tag = None
        last_tag_starts_at = None
        ranges = []
        for count, (topic, msg, time) in enumerate(self.rosbag.read_messages('/tag_multimodal')):
            cur_tag = msg.tag
            if last_tag is None:
                last_tag = cur_tag
                last_tag_starts_at = time

            if cur_tag != last_tag:
                ranges.append((last_tag, (last_tag_starts_at, time)))
                last_tag = cur_tag
                last_tag_starts_at = time
        ranges.append((last_tag, (last_tag_starts_at, time)))

        self._tag_ranges = ranges
        return self._tag_ranges

    @property
    def rosbag(self):
        if hasattr(self, "_rosbag"):
            return self._rosbag
        paths = glob.glob(os.path.join(
            self.folder_path,
            '*.bag',
        ))
        bags = [rosbag.Bag(p) for p in paths] 
        if len(bags) == 0:
            raise Exception("No rosbag in %s"%self.folder_path)
        elif len(bags) > 1:
            raise Exception("More than 1 rosbags in %s"%self.folder_path)
        self._rosbag = bags[0]
        return self._rosbag

if __name__ == '__main__':
    base_path = os.path.dirname(os.path.realpath(__file__))
    er = ExperimentRecord(os.path.join(base_path, 'test_data/experiment_at_2018y04m10d20H40M57S'))
    print er.tag_ranges

