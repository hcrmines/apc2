#!/usr/bin/env python

from apc2.srv import *
# from apc2.msg import *

import rospy
import os
import cv2
import scipy.io as sio
import time
import numpy as np
from classifier_creator import Identifier
import json
import pprint
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from sensor_msgs.msg import PointCloud2
# from geometry_msgs.msg import Point
from std_msgs.msg import Bool
pp = pprint.PrettyPrinter()

def read_json():
    print "loading json"
    json_data = open('apc.json')
    data = json.load(json_data)
    bin_contents = data['bin_contents']
    work_order = data['work_order']
    shelf_contents = {}
    w_o = {}
    for bin_name in bin_contents:
        bin_num = ord(bin_name[-1])-ord('A')
        shelf_contents[bin_num] = set(bin_contents[bin_name])
    print "shelf contents:"
    pp.pprint(shelf_contents)
    print '---'
    for b in work_order:
        bin_num = ord(b["bin"][-1])-ord('A')
        w_o[bin_num]=b["item"] #CHANGE THIS 
    print "work order"
    pp.pprint(w_o)
    return shelf_contents,w_o

bin_points = np.array([
    .78,-.89,.68,
    .51,-1,.7,
    .26,-1.1,.7,
    .79,-.92,.43,
    .52,-1,.44,
    .27,-1.1,.44,
    .77,-.91,.2,
    .52,-1,.2,
    .27,-1.14,.2,
    .77,-.9,-.04,
    .53,-1,-.04,
    .28,-1.15,-.04
    ])
bin_points = np.transpose(bin_points.reshape(12,3))
# print np.shape(bin_points)
# pp.pprint(bin_points)
# pp.pprint( np.transpose(bin_points) - np.arra)

# bin_points = np.transpose(bin_points)
# pp.pprint( bin_points)

# [.46,-.796,-.27]
def point2bin_function(point):

    # shelfTopLeftY = 15 #determine from kinect
    # shelfTopLeftX = 20
    # binWidth = 1 #measure these at the competition
    # binHeight = 1
    # e = 0.05 #tolerance
    # if (x-shelfTopLeftX) > 3*binWidth + e or (y-shelfTopLeftY) > 4*binHeight + e or (x-shelfTopLeftX) < e or (y-shelfTopLeftY) < e:
    #     print 'point is outside of shelf'
    # else:
    #     return int((y-shelfTopLeftY)/binHeight)*3+int((x-shelfTopLeftX)/binWidth)
    p = np.array([[point.x],[point.y],[point.z]])
    # print np.shape(p)
    # print np.sum((bin_points-p)**2,axis =0)
    return np.argmin(np.sum((bin_points-p)**2,axis = 0))
class point:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
# a = point(.5,-1,.7)

# print point2bin_function(a)
class recognition_server:
    def __init__(self):
        self.bridge = CvBridge()
        self.Id = Identifier()
        self.camera = None
        self.shelf_contents,self.work_order = read_json()
        rospy.Subscriber("/cameras/right_hand_camera/image", 
                         Image, self.update_right)

    def update_right(self, update):
        self.camera = update

    def recognize(self, req):
        rval = False
        time.sleep(.5) #wait some time to allow image message to update
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                                self.camera, "bgr8"
                            )
            bin_of_interest = point2bin_function(req.p)
            # if isinstance(bin_of_interest,int):
            bin_contents = self.shelf_contents[bin_of_interest]
            rec_res = self.Id.identify(cv_image,bin_contents)
	    if rec_res >= 0:
           	print 'found ' +  self.Id.items[rec_res], 'want to pick ' + work_order[bin_of_interest]
            	print 'we want to pick <--> ' + str(self.Id.items[rec_res] == work_order[bin_of_interest])
            	rval = self.Id.items[rec_res] == work_order[bin_of_interest]
        except CvBridgeError, e:
            print "Could not convert image message to cv image"
        return Bool(rval)
def main(args):
    r = recognition_server()
    print "initializing"
    rospy.init_node('recognition_server')
    s = rospy.Service('recognize', recognize, r.recognize)
    print "ready to recognize"
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

# a,b =read_json()
# print b[3]

