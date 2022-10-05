import os
import math
import requests
import time
import cv2
import numpy as np

#pos1         = (143.2765395 ,43.32591685)
#pos2         = (143.1873489, 43.28758036)

pos1         = (140.836951 ,38.254853)
pos2         = (140.835100 ,38.253842)

url = "http://cyberjapandata2.gsi.go.jp/general/dem/scripts/getelevation.php" \
       "?lon=%s&lat=%s&outtype=%s" %(pos2[0], pos2[1], "JSON")

resp = requests.get(url, timeout=10)
data = resp.json()


resolution=100000
range_pose_x = int( (pos1[0]-pos2[0])*resolution)
range_pose_y = int( (pos1[1]-pos2[1])*resolution)
print(range_pose_x,range_pose_y)

print(range_pose_x,range_pose_y)
z_array= np.zeros((range_pose_x, range_pose_y))

for i in range(range_pose_x):
    for j in range(range_pose_y):
        #print(j,float(j)/1000,float(j/1000)+pos2[1])
        input_x = pos2[0] + float(i)/resolution
        input_y = pos2[1] + float(j)/resolution
        url = "http://cyberjapandata2.gsi.go.jp/general/dem/scripts/getelevation.php" \
        "?lon=%s&lat=%s&outtype=%s" %(input_x, input_y, "JSON")

        try:
            resp = requests.get(url, timeout=10)
        except:
            print("can not get z info",input_x,input_x)
            z_array[i][j] =0
            continue

        z_array[i][j] = resp.json()["elevation"]
        print(input_x,input_y,resp.json()["elevation"])
        time.sleep(0.2)

header=str(pos2[0]) + "," + str(pos2[1]) + "," + str(range_pose_x) + "," + str(range_pose_y) + "," + str(resolution)
np.savetxt("./test_aoba.txt",z_array, delimiter=",", header=header)

print(data["elevation"])