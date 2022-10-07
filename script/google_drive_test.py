#!/usr/bin/python
 -*- coding: utf-8 -*-
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

        self.gauth.LoadCredentialsFile("/media/data/drone_ws/src/gk_drone_pkg/script/mycreds.txt")
        self.gauth.LoadClientConfigFile("/media/data/drone_ws/src/gk_drone_pkg/script/client_secrets.json")

        if self.gauth.credentials is None:
            self.gauth.LocalWebserverAuth()
        elif self.gauth.access_token_expired:
            self.gauth.Refresh()
        else:
            self.gauth.Authorize()
        self.gauth.SaveCredentialsFile("/media/data/drone_ws/src/gk_drone_pkg/script/mycreds.txt") 
            
        self.drive = GoogleDrive(self.gauth)
 
        rospy.init_node('googledrive_uploader', anonymous=True)
        rospy.Subscriber("chatter", String, self.callback)


 
    def callback(self, String):
        if (self.counter==20):
            f = self.drive.CreateFile({'title' : String.data.split("/")[-1]})


            #f.SetContentFile(os.path.join(path,".png"))
            #f.SetContentFile(os.path.join(path,".txt"))
            f.SetContentFile(String.data)
            
            f['parents'] = [{'id': "1gZehTuWxbyfovkohYqFoew5g1SLLC1re"}]
            f.Upload()
            f = None
            self.counter=0
            print("upload txt to drive")
        else:
            self.counter+=1
        
 
if __name__ == '__main__':
    try:
        de = depth_estimater()
        rospy.spin()
    except rospy.ROSInterruptException: pass

