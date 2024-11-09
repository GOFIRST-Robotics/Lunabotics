#!/bin/bash

ros2 service call /zed2i/zed_node/start_svo_rec zed_interfaces/srv/StartSvoRec "{svo_filename: '/rosbags/test.svo2', compression_mode: 2}"
