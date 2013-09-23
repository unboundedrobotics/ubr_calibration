#!/usr/bin/env python

"""
Compute statistics of LED error
"""

from __future__ import print_function

import rosbag
import argparse
from pylab import *
from math import sqrt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('bagfile', help='The bagfile to analyze.')
args = parser.parse_args()

x = list()
y = list()
z = list()

bag = rosbag.Bag(args.bagfile)

for topic, msg, t in bag.read_messages():
    if topic != 'calibration_data':
        continue

    for obs in msg.world_observations:
        if obs.header.frame_id == 'gripper_led_link':
            x.append(obs.point.x)
            y.append(obs.point.y)
            z.append(obs.point.z)

print(len(x))
bag.close()

avg_x = sum(x)/len(x)
avg_y = sum(y)/len(y)
avg_z = sum(z)/len(z)

print('Average x offset: %f' % avg_x)
print('Average y offset: %f' % avg_y)
print('Average z offset: %f' % avg_z)

x = [k - avg_x for k in x]
y = [k - avg_y for k in y]
z = [k - avg_z for k in z]

d = list()
for k in range(len(x)):
    dist = sqrt(x[k] * x[k] + y[k] * y[k] + z[k] * z[k])
    d.append(dist)


subplot(4, 1, 1)
xlim(-.1, .1)
hist(x, 50)

subplot(4, 1, 2)
xlim(-.1, .1)
hist(y, 50)

subplot(4, 1, 3)
xlim(-.1, .1)
hist(z, 50)

subplot(4, 1, 4)
hist(d, 50)

show()
