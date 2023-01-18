#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

class ArucoDetection :private ArucoDetectionImpl {
public:
	void detect(const cv::Mat & image, Markers& markers);
};
#endif 



