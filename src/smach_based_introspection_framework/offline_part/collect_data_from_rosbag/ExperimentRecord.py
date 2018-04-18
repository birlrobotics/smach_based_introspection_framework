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
            cur_tag = int(msg.tag)
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

    @property
    def unsuccessful_tag_ranges(self):
        if hasattr(self, "_unsuccessful_tag_ranges"):
            return self._unsuccessful_tag_ranges

        unsucc_tag_ranges = []
        for idx, (tag, (start_time, end_time)) in enumerate(self.tag_ranges):
            try:
                next_tag = 0
                forward = 1
                while next_tag == 0:
                    next_tag = self.tag_ranges[idx+forward][0]
                    forward += 1
            except IndexError:
                break
            if tag > 0 and next_tag < 0:
                unsucc_tag_ranges.append((tag, (start_time, end_time)))

        self._unsuccessful_tag_ranges = unsucc_tag_ranges
        return self._unsuccessful_tag_ranges

    @property
    def successful_tag_ranges(self):
        if hasattr(self, "_successful_tag_ranges"):
            return self._successful_tag_ranges

        succ_tag_ranges = []
        for idx, (tag, (start_time, end_time)) in enumerate(self.tag_ranges):
            try:
                next_tag = 0
                forward = 1
                while next_tag == 0:
                    next_tag = self.tag_ranges[idx+forward][0]
                    forward += 1
            except IndexError:
                next_tag = 0
            if tag > 0 and next_tag >= 0:
                succ_tag_ranges.append((tag, (start_time, end_time)))

        self._successful_tag_ranges = succ_tag_ranges
        return self._successful_tag_ranges

if __name__ == '__main__':
    import pprint
    pp = pprint.PrettyPrinter(indent=4)

    base_path = os.path.dirname(os.path.realpath(__file__))
    er = ExperimentRecord(os.path.join(base_path, 'test_data/experiment_at_2018y04m10d20H40M57S'))
    print '\ntag_ranges', '-'*20
    pp.pprint(er.tag_ranges)
    print '\nunsuccessful_tag_ranges', '-'*20
    pp.pprint(er.unsuccessful_tag_ranges)
    print '\nsuccessful_tag_ranges', '-'*20
    pp.pprint(er.successful_tag_ranges)
