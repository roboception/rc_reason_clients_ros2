import copy

def map_custom(msg, rostype):
    if rostype == 'rc_reason_msgs/DetectedTag':
        msg['header'] = {'stamp': msg['timestamp'], 'frame_id': msg['pose_frame']}
        del msg['timestamp']
        del msg['pose_frame']
        msg['tag'] = {'id': msg['id'], 'size': msg['size']}
        del msg['id']
        del msg['size']
        pose = copy.deepcopy(msg['pose'])
        del msg['pose']
        msg['pose'] = {'pose': pose, 'header': msg['header']}
