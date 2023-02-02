#ifndef ARUCO_DETECTION_IMPL_H
#define ARUCO_DETECTION_IMPL_H

//ArucoDetection 瑣碎的程式
class ArucoDetectionImpl {
protected:
	//灰色圖
	cv::Mat gray;
	//邊緣圖
	cv::Mat thresholded;
	//檢查標記候選人照片最外圍是否是黑色
	bool boarderIsBlack(cv::Mat& binaryCode, int fullsize);
	// 建立偵測範圍遮罩於原圖使偵測標記速度加快
	void createMask();
	//篩選標記候選人輪廓是否為接近四邊形且為凸包且大小適當
	bool polyfilter(Poly& candidate, std::vector<cv::Vec4i>& hierarchy, int hierarchyId);
	///增進標記於照片中角點定位精度
	void cornerRefine(Poly& candidate, PolyF& candidatef);
	//讓4個角點順時針排列
	void makePointsOrder(std::vector<cv::Point2i>& points);
	//檢查是否候選輪廓為外輪廓
	bool contourIsOuter(const std::vector<cv::Vec4i>& hierachy, const int idx);
	//建立影像遮罩(偵測範圍)
	void createMask(const cv::Mat&img,const Polys& detectRegions, cv::Mat& masked);
	//比對標記照片的ID，若判斷該照片非標記照片回傳-1
	MarkerId idMatcher(const cv::Mat& warped, PolyF& candidatef);
	//將區域進行正投影仿射
	void warp(PolyF& candidate, cv::Mat& warped);
	void contrastAdjust(const cv::Mat& img, cv::Mat& adjustedImg, const int& contrastFactor);
	int contrastPixelWiseAdjust(const int &val, const int &contrastFactor);

};
#endif