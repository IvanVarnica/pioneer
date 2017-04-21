#!/bin/bash

rostopic pub -r 3 -f ~/acatkin_ws/src/pioneer_simulator/initPose/Alfa_initialPose.yaml /Alfa/initialpose geometry_msgs/PoseWithCovarianceStamped



