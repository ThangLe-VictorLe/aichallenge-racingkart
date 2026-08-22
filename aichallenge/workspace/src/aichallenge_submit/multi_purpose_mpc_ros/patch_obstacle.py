with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/map.py", "r") as f:
    content = f.read()

old_init = """class Obstacle:
    def __init__(self, cx, cy, radius, s=None, e_y=None):
        \"\"\"
        Constructor for a circular obstacle to be placed on a map.
        :param cx: x coordinate of center of obstacle in world coordinates
        :param cy: y coordinate of center of obstacle in world coordinates
        :param radius: radius of circular obstacle in m
        \"\"\"
        self.cx = cx
        self.cy = cy
        self.radius = radius
        self.s = s"""

new_init = """class Obstacle:
    def __init__(self, cx, cy, radius, s=None, e_y=None, vid=None):
        \"\"\"
        Constructor for a circular obstacle to be placed on a map.
        :param cx: x coordinate of center of obstacle in world coordinates
        :param cy: y coordinate of center of obstacle in world coordinates
        :param radius: radius of circular obstacle in m
        :param vid: vehicle ID (for dynamic obstacles)
        \"\"\"
        self.cx = cx
        self.cy = cy
        self.radius = radius
        self.s = s
        self.vid = vid"""

content = content.replace(old_init, new_init)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/map.py", "w") as f:
    f.write(content)
