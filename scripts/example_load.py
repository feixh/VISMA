import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
import argparse
import glob

# protobuf
import protocols.vlslam_pb2 as vlslam_pb2

parser = argparse.ArgumentParser()
parser.add_argument('--dataroot', type=str, required=True,
        help='Root directory to the data folder containing preprocessed data.')
parser.add_argument('--output-dir', type=str, default='tmp')
parser.add_argument('--nodisplay',  action='store_true', default=False,
        help='if set, suppress the visualization loop')

args = parser.parse_args()

def load_edge(filename):
    edge = vlslam_pb2.EdgeMap()
    with open(filename, 'rb') as fid:
        edge.ParseFromString(fid.read())
    edge = np.array(edge.data).reshape((edge.rows, edge.cols))
    return edge

if __name__ == '__main__':
    # handle our weird naming convention
    timestamps = glob.glob(os.path.join(args.dataroot, '*.png'))
    timestamps = [float(os.path.basename(x)[:-4]) for x in timestamps]
    timestamps.sort()

    # load dataset
    dataset = vlslam_pb2.Dataset()
    with open(os.path.join(args.dataroot, 'dataset'), 'rb') as fid:
        dataset.ParseFromString(fid.read())

    plt.ion()
    plt.close('all')
    fig = plt.figure()
    img_plot = fig.add_subplot(121)
    edge_plot = fig.add_subplot(122)

    for i, ts in enumerate(timestamps):
        img_path = os.path.join(args.dataroot, '{:}.000000.png'.format(int(ts)))
        img = plt.imread(img_path)

        edge_path = os.path.join(args.dataroot, '{:}.000000.edge'.format(int(ts)))
        edge = load_edge(edge_path)

        packet = dataset.packets[i]
        gwc = np.array(packet.gwc).reshape((3, 4))
        print('g(world <- camera)=\n{}'.format(gwc))

        if not args.nodisplay:
            img_plot.imshow(img)
            edge_plot.imshow(edge)
            plt.pause(0.01)
