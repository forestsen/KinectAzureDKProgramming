# Kinect Azure DK Programming

## Samples about Kinect Azure DK programming

1. **OpenCV_OneKinect**

   Using OpenCV 4.1.0 function **imshow** to display raw RGB/IR/Depth data acquired from **One** Kinect.

2. **OpenCV_TwoKinects**

   Using OpenCV 4.1.0 function **imshow** to display raw RGB/IR/Depth data acquired from **Two** Kinects on **one** PC.

3. **OpenGL_GLUT_ShowImage**

   Displaying raw RGB/IR/Depth data as the background on the FreeGLUT OpenGL rendering environment.

4. **OpenGL_GLUT_ArUco_AR**

   Using ArUco library to calculate the model view and projection matrix in favor of the rendering Augmented Scene.

5. **OpenGL_GLFW_GLEW_ArUco_AR**

   Using GLFW + GLEW based OpenGL 3 GLSL to display raw RGB data from Kinect on the background and render the ArUco-assisted augmented scene.

6. **OpenGL_GLFW_GLEW_PointCloudRenderer**

   Using GLFW + GLEW based OpenGL 3 GLSL to display raw RGB data from Kinect on the background and render the point cloud transformed from raw Depth data.
   
7. **Aruco_TwoKinects_Calibration_Extrinsics**

   Using ArUco library to calibrate the extrinsic matrix between Two Kinects. We will get two csv files stored two transformation matrix which are **"sub => master"** and **"sub => marker"**.
   
8. **OpenCV_TwoKinects_GreenScreen**

   The code is copied from Azure Kinect SDK example "green screen", however, this project is based on  the OpenCV 4.1.0.

9. **OneKinect_Recording_RGB_DEPTH_IR**

   Record the rgb+depth+ir stream into the mkv video file.

10. **OneKinect_Playback_RGB_DEPTH_IR**

    Playback the mkv video file using opencv.

11. **Open3D_OneKinect**

    Using the Open3D to open the azure kinect device and show the rgb + depth image or the point cloud based on the open3d's visualization class.

## Environment

- Kinect Azure DK driver v1.2.0
- OpenCV 4.1.0
- FreeGLUT 3.0.0
- GLEW 2.1.0
- GLFW 3.3
- GLM 0.9.7.1
- ArUco 3.1.2
- PCL 1.9.1
- Eigen 3.3
- libjpeg-turbo 2.0.3
- libyuv
- Open3D 0.8.0
- Visual Studio Community 2017



## License

Copyright &copy; 2019 Haipeng WANG  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php "MIT License | Open Source Initiative").  

Contact
-------
* Haipeng Wang
    * E-mail: <forestsen@vip.qq.com>

## References

* Kinect2Grabber
  
  https://github.com/UnaNancyOwen/KinectGrabber

* Kinect Azure DK SDK
  
  https://github.com/microsoft/Azure-Kinect-Sensor-SDK



PS: 

I choose the Visual Studio 2017 Win64 compiler to CMake.

On the Visual Studio 2017 platform, the **x64|Release** version is strongly recommended.
