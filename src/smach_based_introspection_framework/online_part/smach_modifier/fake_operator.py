scripted_inputs = [
    'trash',
    'yes',
    'yes',
    'tool_collision',
    'yes',
    'no',
    'trash',
    'yes',
    'yes',
    'object_slip',
    'yes',
    'no',
    'tool_collision',
    'yes',
    'tool_collision',
    'yes',
]

count = 0
def get_input():
    global count
    ret = scripted_inputs[count] 
    print 'fake operator enters \'%s\''%ret
    count += 1
    return ret
    
