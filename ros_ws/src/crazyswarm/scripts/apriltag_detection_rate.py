#!/usr/bin/env python

import sys
import rospy
import csv
from apriltags_ros.msg import AprilTagDetectionArray

class TagStat(object):
    """docstring for TagStat"""
    def __init__(self, tag_id):
        super(TagStat, self).__init__()
        self.id = tag_id
        # self.window_duration = window_duration
        self.global_count = 0
        self.last_seen = None
        self.reset_window()
        self.add()

    def reset_window(self):
        self.period_count = 0
        # self.last_seen = None
        self.max_period = 0.0
        self._sum_period = 0.0
        # self.window_start = rospy.Time.now()
        # print 'tag id=%d window reset %fs' % (self.id, self.window_duration.to_sec())

    def add(self):
        self.global_count += 1
        now = rospy.Time.now()
        # if now > self.window_start + self.window_duration:
        #     self.reset_window()
        if self.last_seen is not None:
            self.period_count += 1
            period = (now - self.last_seen).to_sec()
            self._sum_period += period
            self.max_period = max(self.max_period, period)
        self.last_seen = now

    def hz(self):
        if self._sum_period > 0:
            return float(self.period_count) / self._sum_period
        else:
            return 0

message_count = 0
detection_map = {}
started = False

def hz_callback(event):
    global message_count
    global detection_map
    with open('apriltag_hz.csv', 'a') as csvfile:
        writer = csv.writer(csvfile)
        for tag_id in detection_map:
            tag_stat = detection_map[tag_id]
            print 'id=%d hz=%f max interval=%f percentage=%f' %(
                tag_id,
                tag_stat.hz(),
                tag_stat.max_period,
                tag_stat.global_count/float(message_count))
            writer.writerow([
                tag_id,
                tag_stat.hz(),
                tag_stat.max_period,
                tag_stat.global_count/float(message_count)])
            tag_stat.reset_window()

def callback(data):
    global message_count
    global detection_map
    global started

    if not started:
        if len(data.detections) > 0:
            started = True
            rospy.Timer(rospy.Duration(1.0), hz_callback)
        else:
            return

    message_count += 1
    for detection in data.detections:
        if detection.id in detection_map:
            detection_map[detection.id].add()
        else:
            detection_map[detection.id] = TagStat(detection.id)

    # for tag_id in detection_map:
    #     tag_stat = detection_map[tag_id]
    #     print 'detected %d id=%d hz=%f max interval=%f percentage=%f' %(
    #         len(data.detections),
    #         tag_id,
    #         tag_stat.hz(),
    #         tag_stat.max_period,
    #         tag_stat.global_count/float(message_count))

if __name__ == "__main__":
    rospy.init_node('apriltag_detection_rate')
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback)
    rospy.spin()