#!/usr/bin/env python

# @Author: Jose Rojas
# @Date:   2018-07-10T08:20:17-07:00
# @Email:  jrojas@redlinesolutions.co
# @Project: ros-libav-node
# @Last modified by:   Jose Rojas
# @Last modified time: 2018-07-10T09:07:58-07:00
# @License: MIT License
# @Copyright: Copyright @ 2018, Jose Rojas

import av
import rospy
import time
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
from common import Node

class ScreenCapture(Node):

    def __init__(self):
        rospy.init_node('libav_screen_capture')

        self.stream = None
        self.output = None
        self.seq_num = 0

        image_topic = self.get_param('libav_screen_capture_image_topic')
        video_size = self.get_param('libav_screen_capture_video_size', 'vga')

        self.pub = rospy.Publisher(image_topic, Image, queue_size=1)

        self.input = None
        self.input = video = av.open('', format='x11grab', options={"video_size": video_size})

        stream = next(s for s in video.streams if s.type == 'video')

        for packet in video.demux(stream):
            if rospy.is_shutdown():
                break
            for frame in packet.decode():
                self.on_frame(frame)

    def on_frame(self, frame):
        if self.pub is not None:
            msg = Image()
            msg.data = frame.to_rgb().planes[0].to_bytes()
            msg.width = frame.width
            msg.height = frame.height
            msg.step = frame.width * 3
            msg.is_bigendian = 0
            msg.encoding = 'rgb8'
            msg.header = Header()
            msg.header.seq = self.seq_num
            msg.header.stamp = rospy.get_rostime()
            msg.header.frame_id = "0"

            self.pub.publish(msg)

            self.seq_num += 1

if __name__ == '__main__':
    try:
        ScreenCapture()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
