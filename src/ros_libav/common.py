# @Author: Jose Rojas
# @Date:   2018-07-10T08:20:17-07:00
# @Email:  jrojas@redlinesolutions.co
# @Project: ros-libav-node
# @Last modified by:   jrojas
# @Last modified time: 2018-07-13T17:26:29-07:00
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

        if local is not None:
            return local
        if parent is not None:
            return parent
        if glb is not None:
            return glb

        return default
