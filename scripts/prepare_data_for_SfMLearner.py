import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
import argparse
import glob
import pickle

# protobuf
import vlslam_pb2
import utils

parser = argparse.ArgumentParser()
parser.add_argument('--dataroot',          type=str, required=True,
        help='Root directory to the data folder containing preprocessed data.')
parser.add_argument('--output-dir',        type=str, default='tmp')
parser.add_argument('--ignore-static',     type=int, default=100,
        help='Ignore static frames at the beginning of the sequence')
parser.add_argument('--stride',            type=int, default=5,
        help='time stamp between neighboring target and source frames')
parser.add_argument('--debug',  action='store_true', default=False,
        help='if set, print arrays and show images for inspection')
parser.add_argument('--process-depth',  action='store_true', default=False,
        help='if set and depth files exist, process depth maps')

args = parser.parse_args()

# output image size
OUT_HEIGHT, OUT_WIDTH = 250, 480

def load_items(index, timestamps, dataroot, dataset, stride=1):
    """
    Given a index, load triple images, with associated camera pose, gravity.
    Args:
        index: Index of the target image in dataset.
        timestamps: Timestamp array of images.
        dataroot: Root directory of the dataset.
        dataset: loaded dataset object.
    Returns:
        a list of N images (N=3)
        depth map if exists
        Nx3x4 numpy array of camera poses
        Nx3x3 numpy array of gravity rotation matrix
    """
    def load_single_item(i):
        ts = timestamps[i]
        # ensure the target image has both proceeding & succeeding source images
        img_path = os.path.join(dataroot, '{:}.000000.png'.format(int(ts)))
        img = cv2.imread(img_path)

        # edge_path = os.path.join(dataroot, '{:}.000000.edge'.format(int(ts)))
        # edge = utils.load_edgemap(edge_path)

        packet = dataset.packets[i]
        gwc = np.array(packet.gwc).reshape((3, 4))

        Wg = np.array(list(packet.wg) + [0])
        Rg, _ = cv2.Rodrigues(Wg)
        return img, gwc.astype(np.float32), Rg.astype(np.float32)

    def load_depth(i):
        ts = timestamps[i]
        # ensure the target image has both proceeding & succeeding source images
        depth_path= os.path.join(dataroot, '{:}.000000.depth'.format(int(ts)))
        if os.path.exists(depth_path):
            with open(depth_path, 'rb') as fid:
                h = np.frombuffer(fid.read(4), dtype=np.int32)[0]
                w = np.frombuffer(fid.read(4), dtype=np.int32)[0]
                depth = np.frombuffer(fid.read(), dtype=np.float32)
                depth = depth.reshape([h, w])
                return depth
        else: return None

    images = []
    poses = []
    rotations = []
    for i in [index-stride, index, index+stride]:
        img, gwc, Rg = load_single_item(i)
        images.append(img)
        poses.append(gwc)
        rotations.append(Rg)
    depth = load_depth(index)
    return images, depth, poses, rotations

def resize_and_concat(img, shape=(OUT_HEIGHT, OUT_WIDTH)):
    """ Resize a list of images and concatenate them in horizontal direction.
    Args:
        img: a list of images
        shape: desired image shape [row, col]
    Returns:
        a single numpy array of shape [row, col * N] where N is the number of images in arg img.
    """
    img = [cv2.resize(x, (shape[1], shape[0]), interpolation=cv2.INTER_LINEAR) for x in img]
    return np.concatenate(img, axis=1)

if __name__ == '__main__':
    # handle our weird naming convention
    timestamps = glob.glob(os.path.join(args.dataroot, '*.png'))
    timestamps = [float(os.path.basename(x)[:-4]) for x in timestamps]
    timestamps.sort()

    # load dataset
    dataset = vlslam_pb2.Dataset()
    with open(os.path.join(args.dataroot, 'dataset'), 'rb') as fid:
        dataset.ParseFromString(fid.read())

    # create output directory if not exist
    if not os.path.exists(args.output_dir):
        try:
            os.mkdir(args.output_dir)
            print('folder {} created'.format(args.output_dir))
        except OSError:
            print('failed to create directory @ {}'.format(args.output_dir))
            exit()


    plt.ioff()
    plt.close('all')
    fig = plt.figure()
    img_plot = fig.add_subplot(211)
    depth_plot = fig.add_subplot(212)

    total = len(timestamps)
    for i in range(args.ignore_static, total):
        if i-args.stride >= 0 and i+args.stride < total:
            print('{:6} - [{:6}, {:6}]'.format(i, args.ignore_static, total))
            img, depth, gwc, Rg = load_items(i, timestamps, args.dataroot, dataset, args.stride)
            img = resize_and_concat(img)

            image_path = os.path.join(args.output_dir, '{:06}.jpg'.format(i))
            pose_path = os.path.join(args.output_dir, '{:06}.pkl'.format(i))
            depth_path = os.path.join(args.output_dir, '{:06}_depth.npy'.format(i))

            cv2.imwrite(image_path, img)
            with open(pose_path, 'wb') as fid:
                pickle.dump({'gwc': np.array(gwc), 'Rg': np.array(Rg)}, fid)

            if args.process_depth and depth is not None:
                depth = cv2.resize(depth, (OUT_WIDTH, OUT_HEIGHT), interpolation=cv2.INTER_NEAREST)
                np.save(depth_path, depth)

            if args.debug:
                print(gwc)
                print(Rg)
                img_plot.imshow(img)
                depth_plot.imshow(depth)
                plt.show()
