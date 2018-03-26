
def _search_idx(progs, dmp_cmd_fields):
    result = [None]*len(progs)
    for i, field in enumerate(dmp_cmd_fields):
        for j, prog in enumerate(progs):
            if prog.search(field):
                result[j] = i
                break
    return result

def get_position_xyz_index(dmp_cmd_fields):
    import re
    progs = [
        re.compile(r'position.*\.x'),
        re.compile(r'position.*\.y'),
        re.compile(r'position.*\.z'),
    ]
    return _search_idx(progs, dmp_cmd_fields)

def get_quaternion_xyzw_index(dmp_cmd_fields):
    import re
    progs = [
        re.compile(r'orientation.*\.x'),
        re.compile(r'orientation.*\.y'),
        re.compile(r'orientation.*\.z'),
        re.compile(r'orientation.*\.w'),
    ]
    return _search_idx(progs, dmp_cmd_fields)
