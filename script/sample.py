import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def fetch_tile(z, x, y):
    url = "https://cyberjapandata.gsi.go.jp/xyz/dem/{z}/{x}/{y}.txt".format(z=z, x=x, y=y)
    df =  pd.read_csv(url, header=None).replace("e", 0)
    return df.values

def fetch_all_tiles(north_west, south_east):
    x_range = range(north_west[1], south_east[1]+1)
    y_range = range(north_west[2], south_east[2]+1)
    return  np.concatenate(
        [
            np.concatenate(
                [fetch_tile(north_west[0], x, y) for y in y_range],
                axis=0
            ) for x in x_range
        ],
        axis=1
    )


nabewari = (13, 7262, 3232) 
nabewari_tile = fetch_tile(*nabewari)


plt.imshow(nabewari_tile)
plt.savefig('figure.png') 

tile = fetch_all_tiles((14, 14523, 6464), (14, 14524, 6465))
print(tile.shape) # (512, 512)