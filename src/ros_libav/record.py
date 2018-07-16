#!/usr/bin/env python

# @Author: Jose Rojas
# @Date:   2018-07-10T08:20:17-07:00
# @Email:  jrojas@redlinesolutions.co
# @Project: ros-libav-node
# @Last modified by:   jrojas
# @Last modified time: 2018-07-14T08:52:46-07:00
# @License: MIT License
# @Copyright: Copyright @ 2018, Jose Rojas

import av
import av.filter
import rospy
import datetime
from ros_libav.msg import Stream, StreamEvent
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
from common import Node

class VideoRecorder(Node):

    def __init__(self):
        rospy.init_node('libav_video_recorder', log_level=rospy.INFO)

        self.stream = None
        self.output = None
        self.seq_num = 0
        self.saved_seq_num = 0
        self.last_image = None
        self.stream_events = []
        self.codec = self.get_param('libav_video_encoder_codec', 'libx264')
        self.framerate = self.get_param('libav_video_encoder_framerate', 15)
        self.level = self.get_param('libav_video_encoder_level', 0)
        self.preset = self.get_param('libav_video_encoder_preset', 'ultrafast')
        self.filepath = self.get_param('libav_video_encoder_root_path')
        self.filename = self.get_param('libav_video_encoder_filename', None)
        self.filetype = self.get_param('libav_video_encoder_filetype', "mpg")
        self.bitrate = self.get_param('libav_video_encoder_bitrate', 0)
        self.video_size = self.get_param('libav_video_encoder_video_size')
        self.crop = self.get_param('libav_video_encoder_crop', None)
        self.pixel_format = self.get_param('libav_video_encoder_pixel_format', 'yuv420p')
        self.vflip = self.get_param('libav_video_encoder_flip_v', None)
        self.hflip = self.get_param('libav_video_encoder_flip_h', None)
        delay = self.get_param('libav_video_encoder_delay', 0)

        self.image_topic = image_topic =  self.get_param('libav_video_encoder_image_topic')
        stream_topic = self.get_param('libav_video_encoder_stream_topic')
        stream_event_topic = self.get_param('libav_video_encoder_stream_event_topic')

        self.timer = rospy.Timer(rospy.Duration(1.0/self.framerate), self.on_timer)
        self.pub_stream = rospy.Publisher(stream_topic, Stream, queue_size=0)
        self.pub_stream_event = rospy.Publisher(stream_event_topic, StreamEvent, queue_size=0)
        self.filter_chain = []
        self.filter_graph = av.filter.Graph()

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo('Starting recording in {} seconds...'.format(delay))
        rospy.sleep(delay)
        rospy.loginfo('Recording started after {} seconds'.format(delay))
        self.sub = rospy.Subscriber(image_topic, Image, self.on_image)

        rospy.spin()

    def initialize_stream(self, msg):
        options={}
        if self.codec == 'libx264' or self.codec == 'libx265':
            options['crf'] = str(self.level)
            options['preset'] = self.preset
        filename = self.filename
        if filename == None:
            time_str = datetime.datetime.fromtimestamp(rospy.get_rostime().to_time()).isoformat().replace(":", "_")
            filename = "{}_{}.{}".format(self.image_topic.replace("/", "_"), time_str, self.filetype)

        graph = self.filter_graph
        # you can enumerate available filters with av.filter.filters_available.
        #print(av.filter.filters_available)
        #
        fchain = []

        ow = iw = msg.width
        oh = ih = msg.height


        self.output = av.open("{}/{}".format(self.filepath, filename), 'w', options=options)
        self.stream = self.output.add_stream(self.codec, self.framerate)

        if self.crop is not None or self.vflip is not None or self.hflip is not None:
            fchain.append(graph.add_buffer(width=iw, height=ih, format=self.pixel_encoding(msg)))

        if self.crop is not None:
            fchain.append(graph.add("crop", self.crop))
            fchain[-2].link_to(fchain[-1])

            # scale video
            box = self.crop.split(":")
            rospy.loginfo('cropping to box {}'.format(box))
            #options['crop'] = 'iw={}:ih={}:w={}:h={}:x={}:y={}'.format(msg.width, msg.height, box[2], box[3], box[0], box[1])
            ow = iw = box[0]
            oh = ih = box[1]

        if self.vflip:
            fchain.append(graph.add("vflip"))
            fchain[-2].link_to(fchain[-1])
            rospy.loginfo('flipping vertically')

        if self.hflip:
            fchain.append(graph.add("hflip"))
            fchain[-2].link_to(fchain[-1])
            rospy.loginfo('flipping horizontally')

        if self.video_size is not None:
            # scale video
            shape = self.video_size.split('x')
            rospy.loginfo('scaling to shape {}'.format(shape))
            ow = shape[0]
            oh = shape[1]

        if len(fchain) > 0:
            fchain.append(graph.add("buffersink"))  # graph must end with buffersink...?
            fchain[-2].link_to(fchain[-1])

        self.filter_chain = fchain

        self.stream.bit_rate = self.bitrate
        self.stream.pix_fmt = self.pixel_format
        self.stream.height = int(oh)
        self.stream.width = int(ow)
        assert self.stream.codec_context.is_open == False
        self.stream.codec_context.options = options

        msg = Stream()
        msg.filename = filename
        msg.topic = self.image_topic
        msg.header = Header()
        msg.header.seq = 0
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "0"
        msg.event = self.EVENT_START

        self.filename = filename
        self.pub_stream.publish(msg)

    def terminate_stream(self):
        msg = Stream()
        msg.filename = self.filename
        msg.topic = self.image_topic
        msg.header = Header()
        msg.header.seq = 0
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "0"
        msg.event = self.EVENT_END
        self.pub_stream.publish(msg)

    def on_shutdown(self):
        self.sub.unregister()
        self.timer.shutdown()
        self.terminate_stream()

        rospy.loginfo("Shutting down, saved_seq_num {}".format(self.saved_seq_num))
        delay = self.get_param('libav_video_encoder_finalize_delay', 0)
        rospy.Timer(rospy.Duration(delay), self.on_hard_shutdown)
        rospy.sleep(delay+1)

    def on_hard_shutdown(self, msg):
        if self.output:
            # flush buffers with frame=None
            packet = self.stream.encode(frame=None)
            self.output.mux(packet)
            self.output.close()

    def on_image(self, msg):
        self.last_image = msg
        self.stream_events.append(self.stream_event())

    def on_timer(self, msg):
        if self.last_image is not None:
            if self.stream == None:
                self.initialize_stream(self.last_image)
            self.encode_image(self.last_image)
            self.last_image = None

    def stream_event(self):
        msg = StreamEvent()
        msg.header = Header()
        msg.header.seq = self.seq_num
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "0"
        msg.was_saved = 0
        msg.saved_seq_num = 0
        return msg

    def pixel_encoding(self, msg):
        encoding = None
        if msg.encoding == 'rgb8':
            encoding = 'rgb24'
        elif msg.encoding == 'bgr8':
            encoding = 'bgr24'
        return encoding

    def encode_image(self, msg):

        # publish cached event messages
        for mess in self.stream_events[0:-1]:
            self.pub_stream_event.publish(mess)

        last_evt = self.stream_events[-1]
        last_evt.was_saved = 1
        last_evt.saved_seq_num = self.saved_seq_num
        self.saved_seq_num += 1

        self.stream_events = []

        frame = av.video.VideoFrame(msg.width, msg.height, self.pixel_encoding(msg))
        frame.planes[0].update(msg.data)
        if len(self.filter_chain) > 0:
            self.filter_chain[0].push(frame)
            frame = self.filter_chain[-1].pull()
        packet = self.stream.encode(frame)
        self.output.mux(packet)

        self.pub_stream_event.publish(last_evt)

if __name__ == '__main__':
    try:
        VideoRecorder()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
