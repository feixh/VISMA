# VISMA dataset tools

VISMA stands for Visual Inertial Semantic Mapping and contains both RGB videos and inertial measurements for developing object-level (semantic) mapping systems.

## Install

Requirements
- `OpenCV`: Image I/O and processing. Easy way is to install OpenCV via your favorite package manager. 
- `Eigen`: Linear algebra and matrix manipulation. Install via package manager or build from source.
- `Protobuf`: Utilities for protocol buffer. Install via package manager.
- `folly`: Utilities from facebook. Follow instructions on the github page of folly to install.
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
cmake .. -DCMAKE_INSTALL_PREFIX=../
make -j
```
Then, set `build_evaluation` to `True` in CMakeLists.txt and build.


## Examples

For example usage of data loader, see `example/example_load.cpp` and run `example_load DATASET_DIRECTORY` in the example sub-directory.
