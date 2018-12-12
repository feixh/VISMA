# Minimal OpenGL-based rendering engine for geometric tasks

This is a minimal implementation of OpenGL-based rendering engine for rendering edge map, object silhouette and depth map, which are useful for tasks like model-based tracking, surface normal and/or depth prediction, etc.

For example usage, see `tools/render_depth.cpp` which loads the json configuration and renders a depth map and object mask. To build, turn on the compilation flag in the main CMakeLists. Once build, go to project root and run:

```
bin/render_depth misc/render_depth.json
```

Note, based on the version of OpenGL used by your operating system, you might need to modify the `major_version` and `minor_version` values in `render_depth.json` accordingly. Typically, for relatively new machines, 4 for `major_version` and 3 for `minor_version` should be good. For old machines, 3 for `major_version` and 3 for `minor_version` might work. If you encounter segfault during executation due to OpenGL version, it's most likely due to OpenGL versioning. During initialization, the renderer also prints out the OpenGL version, which provides clues about the proper major/minor version number to set.


## Dependencies
All the extra dependencies (in addition to the ones needed to build the main library) can be installed via the package manager (tested on Ubuntu 16.04).

- OpenGL
- GLUT
- GLEW
- glfw
- glm
