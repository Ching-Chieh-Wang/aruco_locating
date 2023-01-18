#ifndef ARUCO_DETECTION_IMPL_H
#define ARUCO_DETECTION_IMPL_H

//ArucoDetection 瑣碎的程式
class ArucoDetectionImpl {
protected:
	//檢查候選人最外圍是否是黑色
	static bool boarderIsBlack(cv::Mat& binaryCode, int fullsize);
	//讓4個角點順時針排列
	static void makePointsOrder(std::vector<cv::Point2i>& points);
	//檢查是否候選輪廓為外輪廓
	static bool contourIsOuter(const std::vector<cv::Vec4i>& hierachy, const int idx);
	//建立影像遮罩(偵測範圍)
	void createMask(const cv::Mat&img,const Polys& detectRegions, cv::Mat& masked);

	MarkerId idMatcher(const cv::Mat& warped, PolyF& candidatef);

};
#endif