# Aruco locating
An application that use DSLR camera to track the pose(position and orientation) of [ArUco markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) in images, video or live video(not implemented yet).

![aruco_locating](https://github.com/Ching-Chieh-Wang/aruco_locating/assets/81002444/a83f96b0-9a42-4e0e-bb17-f68a89bf1cb4)

# Description:
1. The program [detects ArUco markers](https://www.sciencedirect.com/science/article/abs/pii/S0031320314000235) in image, this program allows user set parameters regarding to the detection algorithm and determine the detection regions on images.
2. Using [IPPE](https://github.com/tobycollins/IPPE) algorithm (A Perspective-n-Point(PnP) Method) to track the pose of detected markers. The global coordinate system is the camera coordinate system. Hence, the origin is at camera center.
3. Using [Bundle Adjustment](https://github.com/HeYijia/MarkerBA) to refine calculated markers pose (Can be turned off).
4. User could visualize and get a tracking result csv file. The csv file conatins marker's IDs, position and pose information, sorted by image.
   
<p align="center">
  <img width="100" height="100" src="https://docs.opencv.org/4.x/marker23.png">
  <br>
  ArUco marker
</p>

# Library Package Installation
1.  [VTK](https://vtk.org/download/): For marker tracking result visualization.
2.  [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) For matrix calculation.
3.  [OpenCV](https://opencv.org/releases/): For ArUco marker detection, pose estimation and image I/O. Opencv Contrib Module is required(viz).
4.  [g2o](https://github.com/RainerKuemmerle/g2o): For Bundle Adjustment.
5.  [MarkerBA](https://github.com/HeYijia/MarkerBA): For marker-based Bundle Adjustment (Optimizing marker pose rather than marker corners positions).
6.  [json](https://github.com/nlohmann/json): For json file I/O.

# How to use:
1.  Aruco marker preparation: [Print](https://tn1ck.github.io/aruco-print/) and Place ArUco markers where you want to track. Fill in marker size (meter) in [config/params.json](https://github.com/Ching-Chieh-Wang/aruco_locating/blob/master/aruco_locating/config/params.json). If having markers with different size, fill in the size with specified ID in "specialMarkerSize". 
2.  [Camera calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html): Using exsited [camera calibration tools](https://www.mathworks.com/help/vision/ref/cameracalibrator-app.html) to obtain camera intrinsic matrix and distortion coeeficient. Fill in camera matrix(kmat) and distortion coefficient(distmat) in [config/params.json](https://github.com/Ching-Chieh-Wang/aruco_locating/blob/master/aruco_locating/config/params.json). During and after this step, the camera auto focus must be turned off.
    <br><img width="250" height="100" src="https://github.com/Ching-Chieh-Wang/aruco_locating/assets/81002444/bdde2874-7b55-413d-9869-cabd2ce9f56e">      
    <img width="350" height="35" src="https://github.com/Ching-Chieh-Wang/aruco_locating/assets/81002444/4c371062-01c8-466c-ae50-288def694d03"><br>
4.  Include files
5.  Declare ArucoLocating
6.  (Optional) Declare ArucoLocating::arucoDetectionSet(source, sourcePath): Adjust the aruco detection algorithm parameters. Recommanded for blurred or scenario that marker is hard to detect.
7.  Declare ArucoLocating::run(source,sourcePath,BA,monitor,parallel):
    * Source: Source::IMG for img file; Source Vidoe for video file
    * sourcePath: folder where the images stored or the path of the video file
    * BA: Turn on/off Bundle Adjustment for optimizing marker pose.
    * monitor: 3D Visualize each img/frame marker detection and tracking result.
    * parallel: Allow Multiprocess for better efficiency. 
8.  Declare ArucoLocating.save(path): Save the marker tracking result as csv file in specificed path.  

```cpp
//4
#include "pch.h"
#include "aruco_locating.h"

int main(){
	ArucoLocating arucoLocator;  //5
	//arucoLocator.arucoDetectionSet(Source::IMAGE, "/imgs"); //6
	arucoLocator.run(Source::IMAGE, "/imgs", true,false, true);  //7
	arucoLocator.save("saved"); //8
}
```

![overlap_elimination](https://github.com/Ching-Chieh-Wang/aruco_locating/assets/81002444/fce0f076-e3e5-4e82-9231-f0045b67ea91)

# Reference
1.  Garrido-Jurado, Sergio, et al. "Automatic generation and detection of highly reliable fiducial markers under occlusion." Pattern Recognition 47.6 (2014): 2280-2292.
2.  https://github.com/HeYijia/MarkerBA/tree/master
3.  Collins, Toby, and Adrien Bartoli. "Infinitesimal plane-based pose estimation." International journal of computer vision 109.3 (2014): 252-286.
4.  高翔, 视觉 SLAM 十四讲: 从理论到实践. 电子工业出版社, 2017.
5.  Graphics Designed By TIM-Chang From <a href="https://zh.lovepik.com/image-401804970/rhinomodeling-camera.html">LovePik.com</a>
