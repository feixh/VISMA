""" Visualize some 2D map stored in the following format
image_height (int32), image_width (int32)
values (dtype, image_height * image_width values in total)
Usage:
    python show_depthmap.py PATH_TO_DEPTH_MAP
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

if __name__ == '__main__':
    filename = sys.argv[1]
    dtype = sys.argv[2]
    assert dtype in ['np.float32', 'np.in32', 'np.uint8']
    with open(filename, 'rb') as fid:
        h = np.frombuffer(fid.read(4), dtype=np.int32)[0]
        w = np.frombuffer(fid.read(4), dtype=np.int32)[0]
        depth = np.frombuffer(fid.read(), dtype=eval(dtype))

        depth = depth.reshape([h, w])
        plt.imshow(depth)
        plt.show()


