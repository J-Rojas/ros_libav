# @Author: Jose Rojas
# @Date:   2018-07-10T08:20:17-07:00
# @Email:  jrojas@redlinesolutions.co
# @Project: ros-libav-node
# @Last modified by:   jrojas
# @Last modified time: 2018-07-13T17:26:29-07:00
# @License: MIT License
# @Copyright: Copyright @ 2018, Jose Rojas

import rospy
import av
import av.filter

class Node(object):

    EVENT_START = 1
    EVENT_END = 2

    def __init__(self, type):
        self.crop = self.get_param('libav_{}_crop'.format(type), None)
        self.vflip = self.get_param('libav_{}_flip_v'.format(type), None)
        self.hflip = self.get_param('libav_{}_flip_h'.format(type), None)
        self.filter_chain = []
        self.filter_graph = None

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

    def pixel_encoding(self, format):
        encoding = None
        if format == 'rgb8':
            encoding = 'rgb24'
        elif format == 'bgr8':
            encoding = 'bgr24'
        return encoding

    def initialize_filter_graph(self, stream, format):

        crop = self.crop
        vflip = self.vflip
        hflip = self.hflip

        graph = av.filter.Graph()
        fchain = []

        ow = iw = stream.width
        oh = ih = stream.height

        if crop is not None or vflip is not None or hflip is not None:
            fchain.append(graph.add_buffer(width=iw, height=ih, format=format))

        if crop is not None:
            fchain.append(graph.add("crop", crop))
            fchain[-2].link_to(fchain[-1])

            # scale video
            box = crop.split(":")
            rospy.logwarn('cropping to box {}'.format(box))
            ow = iw = box[0]
            oh = ih = box[1]

        if vflip:
            fchain.append(graph.add("vflip"))
            fchain[-2].link_to(fchain[-1])
            rospy.loginfo('flipping vertically')

        if hflip:
            fchain.append(graph.add("hflip"))
            fchain[-2].link_to(fchain[-1])
            rospy.loginfo('flipping horizontally')

        if len(fchain) > 0:
            fchain.append(graph.add("buffersink"))  # graph must end with buffersink...?
            fchain[-2].link_to(fchain[-1])

        self.filter_chain = fchain
        self.filter_graph = graph

        return ow, oh

    def process_filter_chain(self, frame):
        if len(self.filter_chain) > 0:
            self.filter_chain[0].push(frame)
            frame = self.filter_chain[-1].pull()
        return frame
