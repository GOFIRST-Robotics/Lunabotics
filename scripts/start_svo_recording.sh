#!/bin/bash

ros2 service call /zed2i/zed_node_zed2i/start_svo_rec zed_msgs/srv/StartSvoRec "{svo_filename: '/rosbags/test.svo2', compression_mode: 2}"
