#!/usr/bin/env python
import rospy
from mixer_node.msg import NodeWeightArray
from geometry_msgs.msg import Transform

inputs = {}

def input_callback_make(name):
    def input_callback(data):
        inputs[name] = [data, inputs[name][1]]
    return input_callback

def controller_callback(data):
    for datum in data.node_weights:
        try:
            inputs[datum.name] = [inputs[datum.name][0], datum.weight]
        except:
            rospy.Subscriber(datum.name, Transform, input_callback_make(datum.name))
            inputs[datum.name] = [0, datum.weight]

def main():
    pub = rospy.Publisher('mixer_out', Transform, queue_size=10)

    rospy.init_node('mixer')
    rate = rospy.Rate(10)

    rospy.Subscriber('autoforce_offb_controller', NodeWeightArray, controller_callback)
    
    while not rospy.is_shutdown():
        total = sum([inputs[key] for key in inputs.keys()])
        out = sum([inputs[key]/total * inputs[key] for key in inputs.keys()])
        pub.publish(out)
        rate.sleep()
