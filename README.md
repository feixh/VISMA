# VISMA dataset tools

VISMA stands for Visual Inertial Semantic Mapping and contains both RGB videos and inertial measurements for developing object-level (semantic) mapping systems.

## Install

Requirements
- `OpenCV`: Image I/O and processing. Easy way is to install OpenCV via your favorite package manager. 
- `Eigen`: Linear algebra and matrix manipulation. Install via package manager or build from source.
- `Protobuf`: Utilities for protocol buffer. Install via package manager.
- `folly`: Utilities from facebook. Follow instructions on the github page of folly to install.
    - Note, to build folly as a shared library, do `cmake .. -DBUILD_SHARED_LIBS=ON -DCXX_CMAKE_FLAGS=-fPIC`
- `Sophus`: Library for SE(3) and SO(3) groups. We provide it in the thirdparty directory. Follow the following instruction to build and install locally.
    - Go to thirdparty/Sophus and `make build`.
    - Go to the newly created build folder and `cmake .. -DCMAKE_INSTALL_PREFIX=../`
    - `make install`
    
    
Once all the requirements are met, make a build directory, enter that directory and `cmake ..` followed by `make`.

To build the evaluation code, you need the following extra dependecies: 
- `Open3D`: For point cloud manipulation and visualization.
- `libigl`: Mesh I/O and geometry processing. This is a header only library, clone the repo into thirdparty directory as libigl.

We provide a version of Open3D in thirdparty directory. First, go to thirdparty/Open3D and follow the instruction on `http://www.open3d.org/docs/getting_started.html#ubuntu` to build, i.e.:
```
util/scripts/install-deps-ubuntu.sh
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=../
make -j
```
Then, set `build_evaluation` to `True` in CMakeLists.txt and build.

## Raw and preprocessed data

Raw data (RGB video and inertial measurements with time stamps) are stored in rosbags. You can run your favorite visual-inertial or visual SLAM to get camera poses. 

In addition to the raw data, we also provide the following preprocessed data:
- RGB images extracted from rosbags (`XXX.png`)
- Associated camera pose at each time instant from our SLAM system (`dataset`)
- Edge map (`XXX.edge`) and
- Object bounding boxes (`XXX.bbox`)

Except the RGB images, all the other data are encoded according to `protocols/vlslam.proto` to avoid custom I/O. This being said, though the data loading example is written in C++, it will not be hard to parse data in other programming frameworks, say, python.


## RGB-D (pseudo) ground truth 

We ran ElasticFusion on RGB-D collected by a Kinect to get a (pseudo)ground truth reconstruction to which our semantic reconstruction is compared.

Folder `RGBD` contains all the data needed to evaluate semantic reconstruction for each sequence. For instance, `clutter1` contains data to evaluate results on `clutter` sequence. `clutter/test.klg.ply` is the point cloud reconstruction of the scene from ElasticFusion. `clutter/fragments` contains the following items:
- `objects.json` contains a complete list of objects in the scene. Each object is named as `XXX_N.ply` where XXX is the object name in the CAD database and N is the count of the object (there might be multiple identical objects in the scene).
- For each object listed in `objects.json`, we have its point cloud segmented out from the RGB-D reconstruction. This is used to find the alignment between semantic reconstruction and RGB-D reconstruction in ICP as described in our paper.
- `alignment.json` contains the ground truth object pose in the scene. The poses are found by orientation constrained ICP described in our paper.
- `augmented_scene.ply` contains RGB-D point cloud with points sampled from the CAD models aligned to the scene. This is a point cloud file since RGB-D reconstruction is in the form of point cloud though the CAD models are provided as meshes.




## Examples

### Data loading

For example usage of data loader, see `example/example_load.cpp` and run `example_load DATASET_DIRECTORY` in the example sub-directory. The input RGB image, pre-computed edge map and object proposals, camera pose from SLAM will be loaded. To load the sparse reconstruction, 2D tracklets of feature points and other information from SLAM, see the protocol buffer file in `protocols/vlslam.proto` and modify the dataset loader accordingly.


### Evaluation

To evaluate the semantic reconstruction, one needs to provide the following paths in the configuration file `cfg/tool.json`:
- Path to the CAD database.
- Path to the RGBD folder described above.
- Dataset to evaluate, e.g., `clutter1`.

We save our semantic mapping results over time in `result.json` file and put it in the dataset folder, say, under `clutter1`. `result.json` contains per time instant semantic reconstruction, where the name of the object models in the CAD database, their pose and id in the SLAM frame are provided.


To evaluate the semantic reconstruction, we first align the semantic reconstruction to the (pseudo-)ground truth RGB-D reconstruction since both reconstructions are upto a global rigid body transformation and live in their own coordinate system. Function `MeshAlignment` is called to align them. Once aligned, the transformation will be written to `result_alignment.json` in the dataset folder. In addition, an augmented view with semantic reconstruction overlaid on the RGB-D reconstruction will be written to  `augmented_view.ply`. The following files will be generated to hold quantitative results:
- `surface_error.json`
- `rotation_error.json`
- `translation_error.json`




