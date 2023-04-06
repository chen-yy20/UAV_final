#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import threading
import random
import numpy as np
from enum import Enum

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# if you can not find cv2 in your python, you can try this. usually happen when you use conda.
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2
import tello_base as tello
from color_detect import find_door

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
        cm = min(100,cm)
        command = "forward "+(str(cm))
        self.control_pub.publish(command)
        print(command)
    
    def back(self, cm):
        cm = min(100,cm)
        command = "back "+(str(cm))
        self.control_pub.publish(command)
        print(command)
    
    def up(self, cm):
        cm = min(100,cm)
        command = "up "+(str(cm))
        self.control_pub.publish(command)
        print(command)
    
    def down(self, cm):
        cm = min(100,cm)
        command = "down "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def right(self, cm):
        cm = min(100,cm)
        command = "right "+(str(cm))
        self.control_pub.publish(command)
        print(command)

    def left(self, cm):
        cm = min(100,cm)
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
    def __init__(self,show_key,state_key):
        rospy.Subscriber("tello_state", String, self.update_state)
        rospy.Subscriber("tello_image", Image, self.update_img)
        self.con_thread = threading.Thread(target = rospy.spin)
        self.con_thread.start()
        self.show_key = show_key
        self.state_key = state_key

    def update_state(self,data):
        global tello_state, tello_state_lock
        tello_state_lock.acquire() #thread locker
        tello_state = data.data
        tello_state_lock.release()
        if self.state_key == 'y':
            print(drone_state())

    def update_img(self,data):
        global img, img_lock
        img_lock.acquire()#thread locker
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding = "passthrough")
        img_lock.release()
        if self.show_key == 'y':
            showimg()
        # print(img)


# mid: -1 为找不到坐标
# return Dict: {mid: -, x:-, y:-, z:-, yaw:-}
def drone_state():
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

# 显示出目前tello拍下的照片
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
        NAVIGATION  = 3 # find the center of locating blanket and adjust tello 
        finished = 6 # task done signal

    def __init__(self , ctrl):
        self.States_Dict = None
        self.ctrl = ctrl
        self.now_stage = self.taskstages.finding_location
  
        self.saved_image = None
        self.move_state = 1 # 初始的方向为90，对应1
        self.img_num = 1
        # 初始化一些完成任务所需要的信息
        self.sleep_time = 4 # 每条指令之间需要间隔sleep_time的时间
        self.fire_detecting_point_right = (6, -212, 170, 90) # 在此tello坐标上寻找着火点
        self.fire_detecting_point_left = (-90, -212, 170, 90) # 在此tello坐标上寻找着火点
        # 窗户的定位 左上0 右上1 左下2 右下3
        self.left_windows = [(-115,-185,185,90),
                             (-90,-185,185,90),
                             (-115,-185,140,90),
                             (-70,-185,144,90)]
        self.right_windows = [(-35,-155,185,90),
                             (18,-185,185,90),
                             (-35,-185,147,90),
                             (25,-185,146,90)]
        # 穿过窗户后从此处开始写死轨迹
        self.navigation_queue = [(-90,-15,70,180), # 2
                                 (105,-22,150,-90), # 1
                                 (90,68,113,180), # 3
                                 (56,160,170,0), # 4
                                 (-84,218,92,-90), # 5
                                 (0,231,90,-90), # destination 
                                 ] 



    def main(self): # main function: examine whether tello finish the task
        while not (self.now_stage == self.taskstages.finished):
            if(self.now_stage == self.taskstages.finding_location):
                self.finding_location()
            elif(self.now_stage == self.taskstages.NAVIGATION):
                self.NAVIGATION()
            elif(self.now_stage == self.taskstages.passing_door):
                self.passing_door()
        self.ctrl.land()
        print("Task Done!")
    
    def finding_location(self): # find locating blanket (the higher, the easier)
        assert (self.now_stage == self.taskstages.finding_location)
        # 寻找定位毯
        while not ( drone_state()['mid'] > 0 ): # if no locating blanket is found:
            print(drone_state())
            distance = random.randint(20,30) # randomly select distance
            # 随机上升一定高度
            if not (self.States_Dict['z'] >= 150):
                self.ctrl.up(distance) # tello up
            time.sleep(1) # wait for command finished
        print("Find locating blanket!")
        self.now_stage = self.taskstages.passing_door

    # target format: [x,y,z,yaw]
    def arrive_target(self,target):
        self.States_Dict = drone_state()
        print("Current State: {}".format(self.States_Dict))
        if self.States_Dict['mid'] < 0 :
            self.now_stage = self.taskstages.finding_location
            print("----------- LOST LOCATION !!! -------------")
            return (False)
        delta_x = self.States_Dict['x']-target[0]
        delta_y = self.States_Dict['y']-target[1]
        delta_z = self.States_Dict['z']-target[2]
        delta_yaw = target[3]-self.States_Dict['yaw']
        result_xyz = False
        result_yaw = False
        if (abs(delta_x)<20 and abs(delta_y)<20 and abs(delta_z)<20) :
            result_xyz = True
        if abs(delta_yaw)<5:
            result_yaw = True
        ans = (result_xyz,result_yaw,delta_x,delta_y,delta_z,delta_yaw)
        return ans

    # target format: [x,y,z,yaw]
    def target_move(self,target):
        sleep_time = self.sleep_time
        # 先调整角度
        self.States_Dict = drone_state()
        yaw = self.States_Dict['yaw']
        if yaw >= -45 and yaw < 45:
            self.move_state = 0
            fixed_yaw = 0
        elif yaw >= 45 and yaw < 135:
            self.move_state = 1
            fixed_yaw = 90
        elif yaw >= 135 or yaw  < -135:
            self.move_state = 2
            fixed_yaw = 180
        elif yaw >= -135 and yaw < -45:
            self.move_state = 3
            fixed_yaw = -90
        self.to_yaw(fixed_yaw)
        print("角度修正:{}->{}".format(yaw,fixed_yaw))

        while (1):
            delta = self.arrive_target(target)
            print("Way to go {}".format(delta))
            if self.now_stage == self.taskstages.finding_location or delta[0]:
                break
            # z方向移动不在乎yaw角
            if delta[4]>20:
                self.ctrl.down(delta[4])
                time.sleep(sleep_time)
            elif delta[4]<-20:
                self.ctrl.up(-delta[4])
                time.sleep(sleep_time)
            # delta[2,3] => x,y
            if self.move_state == 0:
                if delta[2]>20:
                    ctrl.back(delta[2])
                    time.sleep(sleep_time)
                elif delta[2]<-20:
                    ctrl.forward(-delta[2])
                    time.sleep(sleep_time)
                if delta[3]>20:
                    ctrl.right(delta[3])
                    time.sleep(sleep_time)
                elif delta[3]<-20:
                    ctrl.left(-delta[3])
                    time.sleep(sleep_time)
            elif self.move_state == 1:
                if delta[2]>20:
                    ctrl.left(delta[2])
                    time.sleep(sleep_time)
                elif delta[2]<-20:
                    ctrl.right(-delta[2])
                    time.sleep(sleep_time)
                if delta[3]>20:
                    ctrl.back(delta[3])
                    time.sleep(sleep_time)
                elif delta[3]<-20:
                    ctrl.forward(-delta[3])
                    time.sleep(sleep_time)
            elif self.move_state == 2:
                if delta[2]>20:
                    ctrl.forward(delta[2])
                    time.sleep(sleep_time)
                elif delta[2]<-20:
                    ctrl.back(-delta[2])
                    time.sleep(sleep_time)
                if delta[3]>20:
                    ctrl.left(delta[3])
                    time.sleep(sleep_time)
                elif delta[3]<-20:
                    ctrl.right(-delta[3])
                    time.sleep(sleep_time)
            elif self.move_state == 3:
                if delta[2]>20:
                    ctrl.right(delta[2])
                    time.sleep(sleep_time)
                elif delta[2]<-20:
                    ctrl.left(-delta[2])
                    time.sleep(sleep_time)
                if delta[3]>20:
                    ctrl.forward(delta[3])
                    time.sleep(sleep_time)
                elif delta[3]<-20:
                    ctrl.back(-delta[3])
                    time.sleep(sleep_time)

        if not self.now_stage == self.taskstages.finding_location:     
            self.to_yaw(target[3])   
            print("Arrive at position: ",target)
        else:
            print("Return to finding_location!")
        

            



    def get_image(self):
        print("正在拍照，调用get_image()...")
        self.saved_image = img
        # photo_name = str(self.img_num)+".jpg"
        # self.img_num += 1
        # cv2.imwrite(".\\photos\\"+photo_name,img)
        # cv2.imshow("tello_photo",self.saved_image)
        # cv2.waitKey(0)
        print("照片{}拍照完成。".format(self.img_num-1))
        # cv2.imshow("saved_image", self.saved_image)
        # cv2.waitKey()

    def to_yaw(self,target_yaw):     
        delta_yaw = 500
        print("偏航角调整，目标{}".format(target_yaw))
        while abs(delta_yaw) > 5:
            self.States_Dict = drone_state()
            if self.States_Dict['mid'] < 0 :
                self.now_stage = self.taskstages.finding_location
                print("-----------!!! LOST LOCATION !!! -------------")
                return (False)
            delta_yaw = target_yaw - self.States_Dict['yaw']
            if delta_yaw >= 180:
                delta_yaw -= 360
            if delta_yaw <= -180:
                delta_yaw += 360
            if delta_yaw > 5:
                    self.ctrl.ccw(delta_yaw)
            elif delta_yaw <-5:
                self.ctrl.cw(-delta_yaw)
            time.sleep(self.sleep_time)
        print("偏航角调整完成，误差{}".format(delta_yaw))
        return True
    
    def detect_fire(self):
        print("Detecting fire")

    # 计划通过cv进行负反馈调节通过门
    def passing_door(self):
        assert (self.now_stage == self.taskstages.passing_door)
        self.target_move(self.fire_detecting_point_right)
        door_num = find_door(img)
        if door_num:
            print("Door {} detected!".format(door_num))
            self.target_move(self.right_windows[door_num])
            time.sleep(5)
            ctrl.forward(100)
            time.sleep(5)
            ctrl.forward(30)
        else:
            print("No door at right!")

        self.target_move(self.fire_detecting_point_left)
        door_num = find_door(img)
        if door_num:
            print("Door {} detected!".format(door_num))
            self.target_move(self.left_windows[door_num])
            time.sleep(5)
            ctrl.forward(100)
            time.sleep(5)
            ctrl.forward(30)
        else:
            print("No door at left!")
            self.target_move(self.left_windows[1])
            ctrl.forward(100)
            time.sleep(5)
            ctrl.forward(30)
        
        self.now_stage = self.taskstages.NAVIGATION  


    def NAVIGATION(self):
        assert (self.now_stage == self.taskstages.NAVIGATION)
        for point in self.navigation_queue:
            print("========== Next point: {} =============".format(point))
            self.target_move(point)
            self.get_image()
        
        self.now_stage = self.taskstages.finished    

        


if __name__ == '__main__':
    rospy.init_node('tello_control', anonymous=True)
    show_key = raw_input("Show image? input 'y'!")
    state_key = raw_input("Show state? input 'y'!")
    control_pub = rospy.Publisher('command', String, queue_size=1)
    ctrl = control_handler(control_pub)
    infouper = info_updater(show_key,state_key)
    tasker = task_handle(ctrl)
    
    # -212 0 170 90
    # -236 -80 170 90
    time.sleep(2)
    ctrl.mon()
    time.sleep(5)
    while(1):
        if drone_state()['mid'] == -1:
            ctrl.takeoff( )
            print("take off")
            break
    #print("mon")
    time.sleep(2)
    ctrl.up(40)
    print("up 60")
    time.sleep(2)

    tasker.main()
    print("Already out now.")
    exit(0)

    


