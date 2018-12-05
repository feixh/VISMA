project(render)

# glm
add_definitions(-DGLM_SWIZZLE)
add_definitions(-DGLM_FORCE_RADIANS)
add_definitions(-DGLEW_STATIC)
add_definitions(-DGLM_ENABLE_EXPERIMENTAL)

find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${GLFW_INCLUDE_DIRS}
  /usr/lib/x86_64-linux-gnu
)

link_libraries(glfw GL GLEW glut)


add_library(render SHARED renderer.cpp)

add_executable(render_depth tools/render_depth.cpp)
target_link_libraries(render_depth render)
