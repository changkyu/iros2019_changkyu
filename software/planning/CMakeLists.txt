cmake_minimum_required(VERSION 2.8.3)
project(changkyu_planner_pushing)

add_compile_options(-std=c++11)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(Bullet REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OMPL REQUIRED)
find_package(OpenCV REQUIRED)

set(Boost_LIBRARIES boost_program_options boost_system boost_filesystem boost_log)

include_directories(
  include  
  /usr/include/eigen3/
  bullet
  bullet/Utils
  bullet/ThirdPartyLibs
  bullet/ThirdPartyLibs/Glew
  ${BULLET_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

ADD_DEFINITIONS(-DGLEW_STATIC)
ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")
ADD_DEFINITIONS("-DDYNAMIC_LOAD_X11_FUNCTIONS=1")
add_library( bullet_simulation 
  ./bullet/Utils/b3Clock.cpp
  ./bullet/OpenGLWindow/SimpleOpenGL3App.cpp
  ./bullet/OpenGLWindow/X11OpenGLWindow.cpp
  ./bullet/OpenGLWindow/EGLOpenGLWindow.cpp
  ./bullet/OpenGLWindow/fontstash.cpp
  ./bullet/OpenGLWindow/GLInstancingRenderer.cpp
  ./bullet/OpenGLWindow/GLPrimitiveRenderer.cpp
  ./bullet/OpenGLWindow/GLRenderToTexture.cpp
  ./bullet/OpenGLWindow/LoadShader.cpp
  ./bullet/OpenGLWindow/opengl_fontstashcallbacks.cpp
  ./bullet/OpenGLWindow/OpenSans.cpp
  ./bullet/OpenGLWindow/SimpleCamera.cpp
  ./bullet/OpenGLWindow/SimpleOpenGL2App.cpp
  ./bullet/OpenGLWindow/SimpleOpenGL2Renderer.cpp
  ./bullet/OpenGLWindow/SimpleOpenGL3App.cpp
  ./bullet/OpenGLWindow/TwFonts.cpp
  ./bullet/OpenGLWindow/X11OpenGLWindow.cpp
  ./bullet/ThirdPartyLibs/Glew/glew.c
  ./bullet/ExampleBrowser/OpenGLGuiHelper.cpp
  ./bullet/ExampleBrowser/GL_ShapeDrawer.cpp
  ./bullet/ExampleBrowser/CollisionShape2TriangleMesh.cpp
  src/bullet_simulation/collision_simulation.cpp
  src/bullet_simulation/bullet_simulation.cpp
)

set (CMAKE_THREAD_PREFER_PTHREAD TRUE)
FIND_PACKAGE(Threads)
target_link_libraries(bullet_simulation
  ${BULLET_LIBRARIES}    
  Bullet3Common
  dl ${CMAKE_THREAD_LIBS_INIT}
)

add_library(${PROJECT_NAME}
  src/planner.cpp
  src/mdp_planner.cpp
  src/statespace.cpp
  src/kinodynamicRRT.cpp 
)

add_executable(run_expr
  demo/run_expr.cpp
)

add_executable(gen_expr
  demo/gen_expr.cpp
)

add_executable(run_robot
  demo/run_robot.cpp
)

add_executable(show_result
  demo/show_result.cpp
)

add_executable(train_rotator
  demo/train_rotator.cpp
)

target_link_libraries(${PROJECT_NAME}  
  ${OMPL_LIBRARIES}
  ${BULLET_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  bullet_simulation  
  pthread
)

target_link_libraries(run_expr
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  yaml-cpp
)

target_link_libraries(gen_expr
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  yaml-cpp
)

target_link_libraries(run_robot
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  yaml-cpp
)

target_link_libraries(show_result
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  yaml-cpp
)

target_link_libraries(train_rotator
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  yaml-cpp
)
