import os
import math
import requests
import time
import cv2




def long2x_float( long , zoom ):
    
    center_point = 2 ** ( zoom + 7 )
    total_pixels = center_point * 2
    pixels_per_long_degree = total_pixels / 360

    return (center_point + long * pixels_per_long_degree + 0.5) / 256

def lat2y_float( lat , zoom ):
    
    center_point          = 2 ** ( zoom + 7 )
    total_pixels          = center_point * 2
    pixels_per_lng_radian = total_pixels / (2 * math.pi)
    siny = min(max(math.sin(lat * (math.pi / 180)), -0.9999), 0.9999);

    return       (center_point - 0.5 * math.log((1 + siny) / (1 - siny)) * pixels_per_lng_radian + 0.5) / 256

def get_xy_float( long , lat , zoom ):
    
    return long2x_float( long , zoom ) , lat2y_float( lat , zoom )

def make_url_by_xy( x, y, zoom, name ):
    dic = {}
    # https://maps.gsi.go.jp/development/ichiran.html
    dic['cyberjapandata.airphoto'] ='http://cyberjapandata.gsi.go.jp/xyz/airphoto/{z}/{x}/{y}.png'
    dic['cyberjapandata.ort'] = 'http://cyberjapandata.gsi.go.jp/xyz/ort/{z}/{x}/{y}.jpg'
    dic['cyberjapandata.relief'] = 'https://cyberjapandata.gsi.go.jp/xyz/relief/{z}/{x}/{y}.png'
    # https://maps.gsi.go.jp/development/elevation.html 
    # 15
    dic['cyberjapandata.dem5a_png'] = "https://cyberjapandata.gsi.go.jp/xyz/dem5a_png/{z}/{x}/{y}.png"
    # 15
    dic['cyberjapandata.dem5b_png'] = "https://cyberjapandata.gsi.go.jp/xyz/dem5b_png/{z}/{x}/{y}.png"
    # 14
    dic['cyberjapandata.dem_png'] = "https://cyberjapandata.gsi.go.jp/xyz/dem_png/{z}/{x}/{y}.png"
    return dic[name].replace('{z}',str(zoom)).replace('{x}',str(x)).replace('{y}',str(y))

def get_filepath( x , y , zoom , name , dirname ,ext):
    
    cache_dirname = os.path.join( dirname , 'Cache-GetImage', name , str(zoom))

    try:
        os.makedirs(cache_dirname,)
    except:
        print("dir is already exist")

    filename = '%d-%d%s' % ( x , y ,ext)
    filepath = os.path.join( cache_dirname , filename )
    print(filepath)
    return filepath

def get_file( x , y , zoom , dirname, name, ext , sleep_time = 5) :
    
    filepath = get_filepath( x , y , zoom , name, dirname, ext)
    if os.path.exists( filepath ) :
        return filepath

    time.sleep(sleep_time)

    url = make_url_by_xy( x , y , zoom ,name)
    print(url)
    res = requests.get( url )
    with open(filepath,'wb+') as fp:
        fp.write(res.content)
    return filepath

def make_file_by_xy( pos1, pos2, zoom, dirname,  name, ext, out_filepath = None, sleep_time = 5) :        
    
    x1 = min(pos1[0],pos2[0])
    x2 = max(pos1[0],pos2[0])
    y1 = min(pos1[1],pos2[1])
    y2 = max(pos1[1],pos2[1])
    x1 = int(x1)
    x2 = int(x2)
    y1 = int(y1)
    y2 = int(y2)

    img_h = None
    for x in range( x1, x2 +1 ):
        img_v = None
        for y in range( y1, y2 +1 ):
            filepath = get_file( x , y , zoom , dirname, name, ext, sleep_time = 5)
            im1 = cv2.imread( filepath )
            if img_v is None:
                img_v = im1
            else:
                img_v = cv2.vconcat([img_v, im1])
        if img_h is None :
            img_h = img_v
        else:
            img_h = cv2.hconcat([img_h, img_v])

    if out_filepath is not None:
        cv2.imwrite(out_filepath, img_h)
    return img_h

def make_file( pos1, pos2, zoom, out_filepath, name = 'cyberjapandata.ort', ext ='.jpg', max_size = (20,20) , sleep_time = 5) :
    
    dirname , _ = os.path.split(out_filepath)
    long1 = min(pos1[0],pos2[0])
    long2 = max(pos1[0],pos2[0])
    lat1  = min(pos1[1],pos2[1])
    lat2  = max(pos1[1],pos2[1])
    x1,y1 = get_xy_float( long1, lat1, zoom)
    x1_offset = int( ( x1 - int(x1) ) * 256 )
    y2_offset = int( ( y1 - int(y1) ) * 256 )
    x2,y2 = get_xy_float( long2, lat2, zoom)
    x2_offset = int( ( x2 - int(x2) ) * 256 )
    y1_offset = int( ( y2 - int(y2) ) * 256 )

    if x2-x1 > max_size[0] :
        raise Exception( "It's over maxsize (lat) : " , x2-x1 , max_size[0])
    if y1-y2 > max_size[1] :
        raise Exception( "It's over maxsize (lng) : " , y1-y2 , max_size[1])

    im = make_file_by_xy( (x1,y1), (x2,y2), zoom, dirname, name, ext)
    print(im.shape,x1_offset,y1_offset,x2_offset,y2_offset)

    y2_offset = im.shape[0] - 256 + y2_offset
    x2_offset = im.shape[1] - 256 + x2_offset
    cv2.imwrite(out_filepath, im[y1_offset:y2_offset,x1_offset:x2_offset])



pos1         = (143.2765395 ,43.32591685)
pos2         = (143.1873489, 43.28758036)
zoom         = 14
out_filepath = "./output.jpg"
make_file( pos1, pos2, zoom, out_filepath )

out_filepath = "./output_dem.png"
make_file( pos1, pos2, zoom, out_filepath , name='cyberjapandata.dem_png', ext='.png' , max_size=(27,27))

