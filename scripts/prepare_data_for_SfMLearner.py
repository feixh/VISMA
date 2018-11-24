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
parser.add_argument('--debug',  action='store_true', default=False,
        help='if set, print arrays and show images for inspection')

args = parser.parse_args()

def load_triplet(index, timestamps, dataroot, dataset):
    """
    Given a index, load triple images, with associated camera pose, gravity.
    Args:
        index: Index of the target image in dataset.
        timestamps: Timestamp array of images.
        dataroot: Root directory of the dataset.
        dataset: loaded dataset object.
    Returns:
        a list of N images (N=3)
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

    images = []
    poses = []
    rotations = []
    for i in [index-1, index, index]:
        img, gwc, Rg = load_single_item(i)
        images.append(img)
        poses.append(gwc)
        rotations.append(Rg)
    return images, poses, rotations

def resize_and_concat(img, shape=(250, 480)):
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
    img_plot = fig.add_subplot(111)

    total = len(timestamps)
    for i in range(args.ignore_static, total):
        if i > 0 and i+1 < total:
            print('{:6} - [{:6}, {:6}]'.format(i, args.ignore_static, total))
            img, gwc, Rg = load_triplet(i, timestamps, args.dataroot, dataset)
            img = resize_and_concat(img)

            image_path = os.path.join(args.output_dir, '{:06}.jpg'.format(i))
            pose_path = os.path.join(args.output_dir, '{:06}.pkl'.format(i))

            cv2.imwrite(image_path, img)
            with open(pose_path, 'wb') as fid:
                pickle.dump({'gwc': np.array(gwc), 'Rg': np.array(Rg)}, fid)


            if args.debug:
                print(gwc)
                print(Rg)
                img_plot.imshow(img)
                plt.show()
