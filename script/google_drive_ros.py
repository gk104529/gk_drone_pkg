from pydrive.drive import GoogleDrive
from pydrive.auth import GoogleAuth

import os


gauth = GoogleAuth()

gauth.LoadCredentialsFile("mycreds.txt")

if gauth.credentials is None:
    gauth.LocalWebserverAuth()
elif gauth.access_token_expired:
    gauth.Refresh()
else:
    gauth.Authorize()
gauth.SaveCredentialsFile("mycreds.txt") 
       
drive = GoogleDrive(gauth)

path = "/media/data/drone_ws/src/gk_drone_pkg/script/Cache-GetImage/cyberjapandata.dem_png/14/"

for x in os.listdir(path):
    f = drive.CreateFile({'title' : x})
    f.SetContentFile(os.path.join(path,x))
    
    f['parents'] = [{'id': "1gZehTuWxbyfovkohYqFoew5g1SLLC1re"}]
    f.Upload()
    f = None