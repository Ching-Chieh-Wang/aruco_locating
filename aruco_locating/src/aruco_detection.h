#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

class ArucoDetection :private ArucoDetectionImpl {
public:
	void detect(const cv::Mat & image, Markers& markers);
private:
	//原圖灰
	cv::Mat gray;
	//adaptive threshold 後的圖
	cv::Mat thresholded;
	//findContour 得到的所有contour的圖
	cv::Mat imageContours;
	//approxpolyDP 得到的所有四邊形contour的圖
	cv::Mat polyFilteredImg;
	//所有偵測到的marker
	Markers markers;
	// 建立偵測範圍遮罩
	void createMask();
	//篩選contour當中接近四邊形且為凸包且大小超過一定值且過濾大小太近者
	bool polyfilter(Poly& candidate, std::vector<cv::Vec4i>& hierarchy, int i);
	///因為從approxpolyDP得到的角點的最高精度只有像素，套用cornerSubPix得到亞像素角點
	void cornerRefine(Poly& candidate, PolyF& candidatef);
	//將疑似為marker區域進行正投影仿射
	void warp(PolyF& candidate, cv::Mat& warped);
};
#endif 



