#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import threading
import random
import numpy as np
from enum import Enum
from collections import deque

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# if you can not find cv2 in your python, you can try this. usually happen when you use conda.
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2
import tello_base as tello

y_max_th = 200
y_min_th = 170

img = None
tello_state='mid:-1;x:100;y:100;z:-170;mpry:1,180,1;pitch:0;roll:0;yaw:-19;'
tello_state_lock = threading.Lock()    
img_lock = threading.Lock()    

# send command to tello
class control_handler: 
    def __init__(self, control_pub):
        self.control_pub = control_pub
    
    def forward(self, cm):
        command = "forward "+(str(cm))
        self.control_pub.publish(command)
        print(command)
    
    def back(self, cm):
        command = "back "+(str(cm))
        self.control_pub.publish(command)
        print(command)
    
    def up(self, cm):
        command = "up "+(str(cm))
        self.control_pub.publish(command)
        print(command)
    
    def down(self, cm):
        command = "down "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def right(self, cm):
        command = "right "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def left(self, cm):
        command = "left "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def cw(self, cm):
        command = "cw "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def ccw(self, cm):
        command = "ccw "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def takeoff(self):
        command = "takeoff"
        self.control_pub.publish(command)
        print ("ready")
        
    def mon(self):
        command = "mon"
        self.control_pub.publish(command)
        print ("mon")

    def land(self):
        command = "land"
        self.control_pub.publish(command)

    def stop(self):
        command = "stop"
        self.control_pub.publish(command)

#subscribe tello_state and tello_image
class info_updater():   
    def __init__(self):
        rospy.Subscriber("tello_state", String, self.update_state)
        rospy.Subscriber("tello_image", Image, self.update_img)
        self.con_thread = threading.Thread(target = rospy.spin)
        self.con_thread.start()

    def update_state(self,data):
        global tello_state, tello_state_lock
        tello_state_lock.acquire() #thread locker
        tello_state = data.data
        tello_state_lock.release()
        # print(tello_state)

    def update_img(self,data):
        global img, img_lock
        img_lock.acquire()#thread locker
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding = "passthrough")
        img_lock.release()
        # print(img)


# put string into dict, easy to find
def parse_state():
    global tello_state, tello_state_lock
    tello_state_lock.acquire()
    statestr = tello_state.split(';')
    # print (statestr)
    dict={}
    for item in statestr:
        if 'mid:' in item:
            mid = int(item.split(':')[-1])
            dict['mid'] = mid
        elif 'x:' in item:
            x = int(item.split(':')[-1])
            dict['x'] = x
        elif 'z:' in item:
            z = int(item.split(':')[-1])
            dict['z'] = z
        elif 'mpry:' in item:
            mpry = item.split(':')[-1]
            mpry = mpry.split(',')
            dict['yaw'] = int(mpry[1])
        # y can be recognized as mpry, so put y first
        elif 'y:' in item:
            y = int(item.split(':')[-1])
            dict['y'] = y
    tello_state_lock.release()
    return dict

def showimg():
    global img, img_lock
    img_lock.acquire()
    cv2.imshow("tello_image", img)
    cv2.waitKey(2)
    img_lock.release()

# mini task: take off and fly to the center of the blanket.
class task_handle():
    class taskstages(Enum):
        finding_location  = 0 # find locating blanket 
        navigation = 1
        passing_door = 2
        order_location  = 3 # find the center of locating blanket and adjust tello 
        finished = 6 # task done signal

    def __init__(self , ctrl):
        self.States_Dict = None
        self.ctrl = ctrl
        self.now_stage = self.taskstages.finding_location
        self.navigation_queue = deque()

    def main(self): # main function: examine whether tello finish the task
        while not (self.now_stage == self.taskstages.finished):
            if(self.now_stage == self.taskstages.finding_location):
                self.finding_location()
            elif(self.now_stage == self.taskstages.order_location):
                self.order_location()
        self.ctrl.land()
        print("Task Done!")
    
    def finding_location(self): # find locating blanket (the higher, the easier)
        assert (self.now_stage == self.taskstages.finding_location)
        # 寻找定位毯
        while not ( parse_state()['mid'] > 0 ): # if no locating blanket is found:
            distance = random.randint(20,30) # randomly select distance
            print (distance)
            # 随机上升一定高度
            if (self.States_Dict['z'] < 150):
                self.ctrl.up(distance) # tello up
            time.sleep(4) # wait for command finished
            # showimg()
        print("Find locating blanket!")
        self.now_stage = self.taskstages.order_location

    # target format: [x,y,z,yaw]
    def arrive_target(self,target):
        self.States_Dict = parse_state()
        print(self.States_Dict)
        if self.States_Dict['mid'] < 0 :
            self.now_stage = self.taskstages.finding_location
            print("----------- LOST LOCATION !!! -------------")
            return (False)
        delta_x = self.States_Dict['x']-target[0]
        delta_y = self.States_Dict['y']-target[1]
        delta_z = self.States_Dict['z']-target[2]
        delta_yaw = self.States_Dict['yaw']-target[3]
        result = False
        if (abs(delta_x)<15 and abs(delta_y)<15 and abs(delta_z)<15 and abs(delta_yaw)<8):
            result = True
        ans = (result,delta_x,delta_y,delta_z,delta_yaw)
        print(ans)
        return ans

    # target format: [x,y,z,yaw]
    def target_move(self,target):
        sleep_time = 5
        while (1):
            delta = self.arrive_target(target)
            if delta[0]:
                break

            if delta[3]>15:
                self.ctrl.down(delta[3])
                time.sleep(sleep_time)
            elif delta[3]<-15:
                self.ctrl.up(-delta[3])
                time.sleep(sleep_time)
           
            if delta[4] > 8:
                self.ctrl.cw(delta[4])
                time.sleep(sleep_time)
            elif delta[4]<-8:
                self.ctrl.ccw(-delta[4])
                time.sleep(sleep_time)

            if delta[1]>15:
                self.ctrl.left(delta[1])
                time.sleep(sleep_time)
            elif delta[1]<-15:
                self.ctrl.right(-delta[1])
                time.sleep(sleep_time)

            if delta[2]>15:
                self.ctrl.back(delta[2])
                time.sleep(sleep_time)
            elif delta[2]<-15:
                self.ctrl.forward(-delta[2])
                time.sleep(sleep_time)
        print("Arrive at :",target)



        


    # 计划通过cv进行负反馈调节通过门
    def passing_door(self):
        assert (self.now_stage == self.taskstages.passing_door)
        


    def order_location(self):# adjust tello to the center of locating blanket
        assert (self.now_stage == self.taskstages.order_location)
        state_conf = 0
        # 获取当前状态
        # TODO： 定位毯具体坐标
        self.States_Dict = parse_state()
        target = [-100,30,150,90]
        self.target_move(target)
        self.ctrl.stop()
        state_conf += 1
        print("stop")
        showimg()
        self.now_stage = self.taskstages.finished    

        # while not ( self.States_Dict['mpry'][1] <= 8 and self.States_Dict['mpry'][1] >= -8 and self.States_Dict['x'] <= 20 and self.States_Dict['x'] >= -20 and  self.States_Dict['y'] <= 20 and self.States_Dict['y'] >= -20 and abs(self.States_Dict['z']) >= 120 and abs(self.States_Dict['z']) <= 160):
        #     print("in state")
        #     if ( abs(self.States_Dict['z']) > 160 or abs(self.States_Dict['z']) < 120 ):
        #         if (abs(self.States_Dict['z']) < 120):
        #             self.ctrl.up(20)     
        #             time.sleep(4)
        #         elif (abs(self.States_Dict['z']) > 160):
        #             self.ctrl.down(20) 
        #             time.sleep(4)
        #     elif ( self.States_Dict['mpry'][1] < -8 or self.States_Dict['mpry'][1] > 8 ):
        #         if (self.States_Dict['mpry'][1] > 8):
        #             self.ctrl.cw(20)
        #             time.sleep(4)
        #         elif(self.States_Dict['mpry'][1] < -8):
        #             self.ctrl.ccw(20)
        #             time.sleep(4)
        #     elif ( self.States_Dict['x'] < -20 or self.States_Dict['x'] > 20 ):
        #         if (self.States_Dict['x'] < -20):
        #             self.ctrl.forward(20)
        #             time.sleep(4)
        #         elif(self.States_Dict['x'] > 20):
        #             self.ctrl.back(20)
        #             time.sleep(4)
        #     elif ( self.States_Dict['y'] < -20 or self.States_Dict['y'] > 20 ):
        #         if (self.States_Dict['y'] < -20):
        #             self.ctrl.left(20)
        #             time.sleep(4)
        #         elif(self.States_Dict['y'] > 20):
        #             self.ctrl.right(20)
        #             time.sleep(4)
        #     else:
        #         time.sleep(2)
        #         self.ctrl.stop()
        #         state_conf += 1
        #         print("stop")
        #     self.States_Dict = parse_state()
        #     showimg()
        #     if self.States_Dict['mid'] < 0 :
        #         self.now_stage = self.taskstages.finding_location
        #         return
        # self.now_stage = self.taskstages.finished     


if __name__ == '__main__':
    rospy.init_node('tello_control', anonymous=True)

    control_pub = rospy.Publisher('command', String, queue_size=1)
    ctrl = control_handler(control_pub)
    infouper = info_updater()
    tasker = task_handle(ctrl)
    
    time.sleep(2)
    ctrl.mon()
    time.sleep(5)
    while(1):
        if parse_state()['mid'] == -1:
            ctrl.takeoff( )
            print("take off")
            break
    #print("mon")
    time.sleep(4)
    ctrl.up(60)
    print("up 60")
    time.sleep(2)

    tasker.main()

    ctrl.land()

    


