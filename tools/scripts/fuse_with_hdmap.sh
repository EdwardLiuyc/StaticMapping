#!/bin/bash

# get the utm offset of map file file
utm_offset=$(<../../pcd/utm.txt)
echo utm_offset : 
echo $utm_offset

MAP=sampled.pcd
PATH=../../pcd/path.pcd
HDMAP=~/data/hdmap/zhongke_pcd.pcd

../../build/tools/fuse_with_hdmap -map $MAP -path $PATH -hdmap $HDMAP $utm_offset

exit 0