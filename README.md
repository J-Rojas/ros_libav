# Purpose

The purpose of **ros_libav** is to wrap the [libav](http://libav.org) library, a multi-media encoding/decoding toolkit into a ROS Node.

With this package, you can capture audio and video from various sources and encode them with all the available settings in the underlying libav documentation.

# Supported Systems

This module is currently tested and supported on Ubuntu 16.04 and higher.

# Requirements

### Python Modules

* av
* numpy

### Ubuntu packages

* libavfilter-dev
* libavdevice-dev
* libavcodec-dev

# Setup

`sudo apt install libavfilter-dev libavdevice-dev libavcodec-dev`

`pip install -r requirements.txt`

# How-To

See the launch examples for more details.

# License

MIT license
