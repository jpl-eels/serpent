#!/usr/bin/python3

'''
Input: bagfile containing `sensor_msgs/PointCloud2` messages were some messages have timestamps of 0
Output: copy of bagfile, with interpolated timestamps in the `sensor_msgs/PointCloud2` messages
'''

import genpy
import os
import rosbag
# from std_msgs.msg import Header

# Configuration
bag_directory = os.path.expanduser("~") + "/data/jpl_snowboard/"
bag_filename = 'lio_cal_without_images.bag'
pointcloud_topic = '/os_cloud_node/points'
pointcloud_period = genpy.Duration(0, 1e8)
save_filename = 'fixed_points_' + bag_filename

# Open bag
bag = rosbag.Bag(bag_directory + bag_filename, 'r')
print("Bagfile '{}' opened.\n\t{:20}: {}\n\t{:20}: {}\n\t{:20}: {} s\n\t{:20}: {}\n\t{:20}: {}".format(bag._filename,\
    "Start Time", bag.get_start_time(),\
    "End Time", bag.get_end_time(),\
    "Duration", bag.get_end_time() - bag.get_start_time(),\
    "Message Count", bag.get_message_count(),\
    "Type/Topic Information", bag.get_type_and_topic_info()))

# Iterate over messages, save to new bag with addition of imu messages
new_bag = rosbag.Bag(bag_directory + save_filename, mode='w')
last_timestamp = genpy.Time()
corrected_count = 0
for topic, msg, t in bag.read_messages([]):
    if topic in pointcloud_topic:
        try:
            # print(msg.header.stamp)
            if msg.header.stamp == genpy.Time():
                msg.header.stamp = last_timestamp + pointcloud_period
                corrected_count += 1
            last_timestamp = msg.header.stamp
        except:
            print("Failed to correct pointcloud message")
    new_bag.write(topic, msg, t)
print("Finished generating new bag. Corrected {} pointcloud timestamps".format(corrected_count))

# Close bag
bag.close()
new_bag.close()
print("Closed bagfiles")
