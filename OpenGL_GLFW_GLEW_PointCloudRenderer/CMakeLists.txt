cmake_minimum_required(VERSION 2.8.3)

if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "/Z7")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/INCREMENTAL:NO /DEBUG /OPT:REF /OPT:ICF")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4100 /wd4127 /wd4150 /wd4191 /wd4242 /wd4244 /wd4251 /wd4305 /wd4365 /wd4512 /wd4514 /wd4548 /wd4571 /wd4619 /wd4625 /wd4626 /wd4628 /wd4668 /wd4710 /wd4711 /wd4738 /wd4820 /wd4946 /wd4986 /wd4819 /wd4996 /wd4267")
    add_definitions(-DNDEBUG)
	add_definitions(-D_USE_MATH_DEFINES)
    add_definitions(-DNOMINMAX)

    add_definitions(-DUSE_GL_3)
    # Tell MSVC to use main instead of WinMain for Windows subsystem executables
    set_target_properties(${WINDOWS_BINARIES} PROPERTIES LINK_FLAGS "/ENTRY:mainCRTStartup")
    add_definitions(-DGLEW_STATIC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    
endif()

INCLUDE_DIRECTORIES(${K4A_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${ArUco_INCLUDE_DIRS})
include_directories(${GLM_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${GLFW_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
add_executable(GLFW_GLEW_PointCloudRenderer "Source_gl3_point_cloud.cpp" "../Util.cpp" "../StaticImageProperties.cpp" "../PointCloudRenderer.cpp" "../Shader.cpp" "../Texture.cpp" "../VideoRenderer.cpp")
target_link_libraries(GLFW_GLEW_PointCloudRenderer opengl32.lib glu32.lib ${OpenCV_LIBS} ${GLFW_LIBRARIES} ${GLEW_LIBRARY} ${ArUco_LIBRARIES} ${K4A_LIBS})