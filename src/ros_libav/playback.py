#!/usr/bin/env python

# @Author: Jose Rojas
# @Date:   2018-07-10T08:20:17-07:00
# @Email:  jrojas@redlinesolutions.co
# @Project: ros-libav-node
# @Last modified by:   jrojas
# @Last modified time: 2018-07-13T18:51:19-07:00
# @License: MIT License
# @Copyright: Copyright @ 2018, Jose Rojas

import av
import rospy
import datetime
from ros_libav.msg import Stream, StreamEvent
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
from common import Node

class VideoPlayer(Node):

    def __init__(self):
        rospy.init_node('libav_video_player')

        self.stream = None
        self.output = None
        self.saved_seq_num = -1
        self.filepath = self.get_param('libav_video_decoder_root_path')
        stream_topic = self.get_param('libav_video_decoder_stream_topic')
        stream_event_topic = self.get_param('libav_video_decoder_stream_event_topic')

        self.sub_stream = rospy.Subscriber(stream_topic, Stream, self.on_stream)
        self.sub_stream_event = rospy.Subscriber(stream_event_topic, StreamEvent, self.on_stream_event)
        self.generator = None

        rospy.on_shutdown(self.on_shutdown)

        rospy.spin()

    def initialize_stream(self, msg):
        if msg.event == self.EVENT_START:
            topic = msg.topic
            filename = msg.filename
            rospy.loginfo("initialize_stream")
            self.saved_seq_num = -1
            self.output = av.open("{}/{}".format(self.filepath, filename), 'r')
            self.stream = next(s for s in self.output.streams if s.type == 'video')
            self.generator = self.next_frame()

    def on_shutdown(self):
        self.sub_stream.unregister()
        self.sub_stream_event.unregister()

    def on_stream(self, msg):
        self.pub = rospy.Publisher(msg.topic, Image, queue_size=0)
        self.initialize_stream(msg)

    def on_stream_event(self, msg):
        if self.stream is None or self.generator is None:
            rospy.sleep(0.1)
        if msg.was_saved:
            frame_index = msg.saved_seq_num
            rospy.loginfo("frame_index {}, saved_seq_num {}".format(frame_index, self.saved_seq_num))
            assert frame_index == self.saved_seq_num + 1
            self.saved_seq_num = frame_index
            frame = self.generator.next()
            self.decode_image(frame, msg)

    def next_frame(self):
        for packet in self.output.demux(self.stream):
            for frame in packet.decode():
                yield frame

    def on_image(self, msg):
        self.last_image = msg
        self.stream_events.append(self.stream_event(False))

    def decode_image(self, frame, orig_msg):
        msg = Image()
        msg.data = frame.to_rgb().planes[0].to_bytes()
        msg.width = frame.width
        msg.height = frame.height
        msg.step = frame.width * 3
        msg.is_bigendian = 0
        msg.encoding = 'rgb8'
        msg.header = Header()
        msg.header = orig_msg.header

        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        VideoPlayer()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
