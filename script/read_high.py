import os
import math
import requests
import time
import cv2


def get_tile_bbox(zoom, topleft_x, topleft_y, size_x=1, size_y=1):
    """
    https://tools.ietf.org/html/rfc7946#section-5
    """
    def num2deg(xtile, ytile, zoom):
        # https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Python
        n = 2.0 ** zoom
        lon_deg = xtile / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
        lat_deg = math.degrees(lat_rad)
        return (lon_deg, lat_deg)
    
    right_top = num2deg(topleft_x + size_x , topleft_y, zoom)
    left_bottom = num2deg(topleft_x, topleft_y + size_y, zoom)
    return (left_bottom[0], left_bottom[1], right_top[0], right_top[1])

def calc_height_chiriin_style(R, G, B, u=100):
    hyoko = int(R*256*256 + G * 256 + B)
    if hyoko == 8388608:
        raise ValueError('N/A')
    if hyoko > 8388608:
        hyoko = (hyoko - 16777216)/u
    if hyoko < 8388608:
        hyoko = hyoko/u
    return hyoko

def plot_height(tile, index, direction, z, x, y, xtile_size=1, ytile_size=1, u=1):

    bbox = get_tile_bbox(z, x, y, xtile_size, ytile_size) 
    width = abs(bbox[2] - bbox[0])
    height = abs(bbox[3] - bbox[1])
    if direction == 0: 
        line = tile[:,index]
        per_deg = -1 * height/tile.shape[0]
        fixed_deg = width/tile.shape[1] * index + bbox[0]
        deg = bbox[3]
        title = 'height at {} degrees longitude'.format(fixed_deg)
    else:
        line = tile[index,:]
        per_deg = width/tile.shape[1]
        fixed_deg = -1 * height/tile.shape[0] * index + bbox[3]
        deg = bbox[0]
        title = 'height at {} degrees latitude'.format(fixed_deg)
        
    heights = []
    heights_err = []
    degrees = []
    for k in range(len(line)):
        R = line[k,0]
        G = line[k,1]
        B = line[k,2]
        try:
            heights.append(calc_height_chiriin_style(R, G, B, u))
        except ValueError as e:
            heights.append(0)
            heights_err.append((i,j))
        degrees.append(deg)
        deg += per_deg

    fig, ax= plt.subplots(figsize=(8, 4)) 
    plt.plot(degrees, heights)
    plt.title(title)
    ax.text(0.01, 0.95, 'Errors: {}'.format(len(heights_err)), transform=ax.transAxes)
    plt.show()



print(get_tile_bbox(15,3676,1499))
