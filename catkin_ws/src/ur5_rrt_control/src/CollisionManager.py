import yaml

import rospy

class CollisionManager(object):
"""CollisionManager allows for checking if the robot is in collision with an obstacle or itself.

:param param1: description
"""

    __init__(self):
        pass

class CollisionBody(object):
"""A collection of collision sphere poses relative to an object frames. The object's name is also stored and can be combined and separated from other CollisionBody objects.

:param param1: description
"""
    __init__(self, path_to_file):
        self._frame_to_spheres = {}
        with open(path_to_file, "r") as f:
            config = yaml.safe_load(f)
            if "group_id" not in config:
                rospy.logerr("[CollisionBody] {} is missing 'group_id'!".format(path_to_file)
                return False
            if "frames" not in config:
                rospy.logerr("[CollisionBody] {} is missing 'frames'!".format(path_to_file))
                return False
            self._group_id = config["group_id"]
            frames = config["frames"]
            for i, frame in config["frames"]:
                if "frame_id" not in frame:
                    rospy.logerr("[CollisionBody] {} is missing 'frame_id'!".format(path_to_file))
                    return False
                if "pos" not in frame or len(frame["pos"]) != 3:
                    rospy.logerr("[CollisionBody] sphere with frame id {} in {} is missing 'pos' or is incorrect length!".format(frame_id, path_to_file))
                    return False
                if "radius" not in frame:
                    rospy.logerr("[CollisionBody] sphere with frame id {} in {} is missing 'radius'".format(frame_id, path_to_file))
                    return False
        return True
        
CollisionSphere(object):
    __init__(self, group_id, frame_id, pos, radius):
        self._group_id = group_id
        self._frame_id = frame_id
        self._pos = pos
        self._radius = radius
