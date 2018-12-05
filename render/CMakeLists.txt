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
  ${CMAKE_SOURCE_DIR}/thirdparty/glad/include
  ${GLFW_INCLUDE_DIRS}
)

link_libraries(
  glfw
  GL)


add_library(render SHARED renderer.cpp)
