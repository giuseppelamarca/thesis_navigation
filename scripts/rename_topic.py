from rosbag import Bag
import sys

in_bag = sys.argv[1]
out_bag = sys.argv[2]
topic_old = sys.argv[3]
topic_new = sys.argv[4]

with Bag(out_bag, 'w') as out: 
    for topic, msg, t in Bag(in_bag):
        out.write(topic_new if topic == topic_old else topic, msg, t)
