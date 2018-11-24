import numpy as np
import vlslam_pb2

def load_edgemap(filename):
    edge = vlslam_pb2.EdgeMap()
    with open(filename, 'rb') as fid:
        edge.ParseFromString(fid.read())
    edge = np.array(edge.data).reshape((edge.rows, edge.cols))
    return edge


