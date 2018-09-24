# VISMA dataset tools

VISMA stands for Visual Inertial Semantic Mapping and contains both RGB videos and inertial measurements for developing object-level (semantic) mapping systems.

## Install

Requirements
- `OpenCV`: Image I/O and processing. Easy way is to install OpenCV via your favorite package manager. 
- `Eigen`: Linear algebra and matrix manipulation. Install via package manager or build from source.
- `libigl`: Mesh I/O and geometry processing. Header only library, clone the repo and install.
- `Protobuf`: Utilities for protocol buffer. Install via package manager.
- `folly`: Utilities from facebook. Follow instructions on the github page of folly to install.
- `Sophus`: Library for SE(3) and SO(3) groups. We provide it in the thirdparty directory. Follow the following instruction to build and install locally.
    - Go to thirdparty/Sophus and `make build`.
    - Go to the newly created build folder and `cmake .. -DCMAKE_INSTALL_PREFIX=../`
    - `make install`
    
    
Once all the requirements are met, make a build directory, enter that directory and `cmake ..` followed by `make`.
