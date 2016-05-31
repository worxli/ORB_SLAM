import rosbag

num_msgs = 100
nth = 10

i = 0
with rosbag.Bag('output.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('fisheye.bag').read_messages():
        if num_msgs < 1:
            break
        num_msgs -= 1
        i += 1
        if i % 5 == 0:
            outbag.write(topic, msg, t)