# save into .txt files
rostopic echo -b rosbag.bag -p /topic/sub_topic1 > topic1.txt
rostopic echo -b rosbag.bag -p /topic/sub_topic2 > topic2.txt
rostopic echo -b rosbag.bag -p /topic/sub_topic3 > topic3.txt
