#!/usr/bin/python

import rospy
from rover_camera.srv import *
import time

import subprocess


def call_launch_file(cam_name, device_file, output_topic=None):
    # log = open("/home/artemis" + cam_name, 'w')
    log = None
    device = "device:=" + device_file
    camera = "name:=" + cam_name
    if output_topic is not None:
        topic = "topic:=True"
        topic_name = "topic_name:=" + output_topic
    else:
        topic = "topic:=False"
        topic_name = "topic_name:=default"

    process = subprocess.Popen([
        "roslaunch",
        "rover_camera",
        "uvc.launch",
        device,
        camera,
        topic,
        topic_name], stdout=log, stderr=log)

    message = "started node for camera " + device_file
    rospy.loginfo(message)

    return process.pid


def kill_process(pid):
    rospy.logwarn("Killing pid " + str(pid))

    if pid is not 0:
        pid_str = str(pid)
        subprocess.Popen(["kill", pid_str])


class FeedSwitcher():
    def __init__(self):

        self.param_list = ["/camera/wide_angle", "/camera/arm", "/camera/pan_tilt"]
        self.topic_map = {}
        self.device_map = {}

        self.load_topic_names()

        self.current_pid = 0

        self.current_feed = 0

        rospy.init_node("feed_switcher", anonymous=False)
        s = rospy.Service('changeFeed', ChangeFeed, self.handle_request)

        rospy.loginfo('feed_switcher ready')

    def handle_request(self, req):
        desired_camera = req.camera_name
        print desired_camera

        try:
            desired_device = self.device_map[desired_camera]
            desired_topic = self.topic_map[desired_camera]
        except KeyError:
            rospy.logerr("Invalid device parameter " + desired_camera)
            return ChangeFeedResponse(0, self.current_pid)

        kill_process(self.current_pid)
        time.sleep(0.2)
        self.current_pid = call_launch_file(desired_camera, desired_device, desired_topic)

        success = 1

        return ChangeFeedResponse(success, self.current_pid)
        pass

    # def new_handle_change_feed(self, req):
    #
    # desired_screen_id = req.screenId
    #     desired_feed_id = req.feedId
    #
    #     try:
    #         other_screen_id = self.feed_map[desired_feed_id]
    #     except KeyError:
    #         other_screen_id = None
    #
    #     if other_screen_id is None:
    #         # traditional switch
    #         if self.process_map[desired_screen_id] is not 0:
    #             kill_process(self.process_map[desired_screen_id])
    #             for i in self.feed_map:
    #                 if self.feed_map[i] is desired_screen_id:
    #                     self.feed_map.pop(i)
    #                     break
    #         time.sleep(0.1)
    #         self.process_map[desired_screen_id] = call_launch_file(self.camera_name[desired_feed_id],
    #                                                                self.camera_list[desired_feed_id],
    #                                                                self.topic_list[desired_screen_id])
    #         print("launched process with pid: ", self.process_map[desired_screen_id])
    #         self.feed_map[desired_feed_id] = desired_screen_id
    #
    #     else:
    #         # screen interchange
    #         other_feed_id = None
    #         for i in self.feed_map:
    #             if self.feed_map[i] is desired_screen_id:
    #                 other_feed_id = i
    #                 break
    #
    #         try:
    #             kill_process(self.process_map[other_screen_id])  # kill the already assigned pid
    #             for i in self.feed_map:
    #                 if self.feed_map[i] is other_screen_id:
    #                     self.feed_map.pop(i)
    #                     break
    #
    #             kill_process(self.process_map[desired_screen_id])  # kill the target screen
    #             for i in self.feed_map:
    #                 if self.feed_map[i] is desired_screen_id:
    #                     self.feed_map.pop(i)
    #                     break
    #
    #         except KeyError:
    #             pass
    #         time.sleep(0.1)
    #         self.process_map[desired_screen_id] = call_launch_file(self.camera_name[desired_feed_id],
    #                                                                self.camera_list[desired_feed_id],
    #                                                                self.topic_list[desired_screen_id])
    #         self.feed_map[desired_feed_id] = desired_screen_id
    #
    #         if other_feed_id is not None:
    #             self.process_map[other_screen_id] = call_launch_file(self.camera_name[other_feed_id],
    #                                                                  self.camera_list[other_feed_id],
    #                                                                  self.topic_list[other_screen_id])
    #             self.feed_map[other_feed_id] = other_screen_id
    #
    #     return ChangeFeedResponse(self.process_map[desired_screen_id], None)

    def load_topic_names(self):
        for item in self.param_list:
            # get topic from parameter server for all camera in list
            temp = rospy.get_param(item)
            if temp is not None:
                self.topic_map[item] = temp

            # get device file from parameter server
            temp = rospy.get_param(item + "_dev")
            if temp is not None:
                self.device_map[item] = temp

        print(self.device_map)
        print(self.topic_map)

    def close(self):
        print
        print "Called close"
        kill_process(self.current_pid)
        exit(0)


switcher = FeedSwitcher()
rospy.spin()
switcher.close()
