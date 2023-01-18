#ifndef ARUCO_DETECTION_SETTING_H
#define ARUCO_DETECTION_SETTING_H

class ArucoDetectionSetting :private ArucoDetectionImpl {
public:
	void run(const Source source, const std::string& sourcePath);
	bool tune(const FrameNumber frameNumber, std::unique_ptr<cv::Mat> img, const std::string& imgName);
	Markers markers();
private:
	//原圖
	cv::Mat img;
	//灰色圖
	cv::Mat gray;
	//邊緣圖
	cv::Mat thresholded;
	//偵測到的輪廓
	cv::Mat contoursImg;
	//角點定位優化結果
	cv::Mat cornerRefineImg;
	//找到的所有輪廓
	Polys contours;
	//疑似標記者
	Polys candidates;
	//輪廓階級
	std::vector<cv::Vec4i> contourHierarchy;
	//調整圖片偵測邊緣的參數
	static void setThreshParams(int, void*);
	//調整多邊形的性質篩選
	static void setPolyParams(int ,void*);
	//調整角點定位優化的搜索視窗大小
	static void setCornerRefineParams(int, void*);
	//將區域進行正投影仿射
	static void warp(PolyF& candidate, cv::Mat& warped, const cv::Mat& gray);
	void save();
	Markers _markers;
};


#endif 




