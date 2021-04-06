#!/usr/bin/python3
import re
import os
import sys
import struct
import numpy
import scipy.io
import rospy
import rosbag
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
import progressbar


def mat2pointcloud(filename):
    m = scipy.io.loadmat(filename)
    scan = numpy.transpose(m['SCAN']['XYZ'][0][0]).astype(numpy.float32)
    stamp = m['SCAN']['timestamp_laser'][0][0][0][0] * 1e-6

    cloud = PointCloud2()
    cloud.header.stamp = rospy.Time.from_sec(stamp)
    cloud.header.frame_id = 'velodyne'
    cloud = pc2.create_cloud_xyz32(cloud.header, scan)
    return cloud


def main():
    if len(sys.argv) < 2:
        print('usage: ford2bag.py src_dirname output_filename')

    output_filename = sys.argv[1]

    rospy.init_node('bag')
    filenames = sorted(
        ['SCANS/' + x for x in os.listdir('SCANS') if re.match('Scan[0-9]*\.mat', x)])

    progress = progressbar.ProgressBar(maxval=len(filenames))
    progress.start()

    # progress = progressbar.ProgressBar(max_value=len(filenames))
    pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=32)
    with rosbag.Bag(output_filename, 'w') as bag:
        for i, filename in enumerate(filenames):
            if rospy.is_shutdown():
                break
            progress.update(i)
            cloud = mat2pointcloud(filename)
            if pub.get_num_connections():
                pub.publish(cloud)
            bag.write('/velodyne_points', cloud, cloud.header.stamp)


if __name__ == '__main__':
    main()
