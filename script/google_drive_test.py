#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

from pydrive.drive import GoogleDrive
from pydrive.auth import GoogleAuth
import rospy
import os
import sys
import message_filters
from sensor_msgs.msg import Image
import rosparam
from std_msgs.msg import String

class depth_estimater:
 
    def __init__(self):
        self.counter=0
        self.gauth = GoogleAuth()

        self.gauth.LoadCredentialsFile("/home/dji/osdk_ros_ws/src/gk_drone_pkg/script/mycreds.txt")
        self.gauth.LoadClientConfigFile("/home/dji/osdk_ros_ws/src/gk_drone_pkg/script/client_secrets.json")

        if self.gauth.credentials is None:
            self.gauth.LocalWebserverAuth()
        elif self.gauth.access_token_expired:
            self.gauth.Refresh()
        else:
            self.gauth.Authorize()
        self.gauth.SaveCredentialsFile("/home/dji/osdk_ros_ws/src/gk_drone_pkg/script/mycreds.txt") 
            
        self.drive = GoogleDrive(self.gauth)
 
        rospy.init_node('googledrive_uploader', anonymous=True)
        rospy.Subscriber("save_file", String, self.callback)


 
    def callback(self, String):
        #if (self.counter==20):
        f_png = self.drive.CreateFile({'title' : String.data.split("/")[-1]+".png"})
        path_png = String.data + ".png"
        f_png.SetContentFile(path_png)
        f_png['parents'] = [{'id': "1gZehTuWxbyfovkohYqFoew5g1SLLC1re"}]
        f_png.Upload()
        f_png = None
        
        f_csv = self.drive.CreateFile({'title' : String.data.split("/")[-1]+".csv"})
        path_csv = String.data + ".csv"
        f_csv.SetContentFile(path_csv)
        f_csv['parents'] = [{'id': "1gZehTuWxbyfovkohYqFoew5g1SLLC1re"}]
        f_csv.Upload()
        f_csv = None
        
        #f.SetContentFile(path_txt)
        #f.SetContentFile(String.data)
        
        
        #self.counter=0
        print("upload txt to drive")
        #else:
        #    self.counter+=1
        
 
if __name__ == '__main__':
    try:
        de = depth_estimater()
        rospy.spin()
    except rospy.ROSInterruptException: pass

