#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include "marker.h"
#include "aruco_detection_impl.h"

class ArucoDetection :private ArucoDetectionImpl {
public:
	void detect(const cv::Mat & image, Markers& markers);
};
#endif 


