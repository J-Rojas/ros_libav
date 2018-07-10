# @Author: Jose Rojas
# @Date:   2018-07-10T08:20:17-07:00
# @Email:  jrojas@redlinesolutions.co
# @Project: ros-libav-node
# @Last modified by:   Jose Rojas
# @Last modified time: 2018-07-10T09:08:18-07:00
# @License: MIT License
# @Copyright: Copyright @ 2018, Jose Rojas

import rospy

class Node:

    EVENT_START = 1
    EVENT_END = 2

    def get_param(self, name, default=None):

        local = rospy.get_param("~{}".format(name), None)
        parent = rospy.get_param("{}".format(name), None)
        glb = rospy.get_param("/{}".format(name), None)

        if glb:
            return glb
        if parent:
            return parent
        if local:
            return local

        return default
