#include "aruco_detection_setting.h"



void ArucoDetectionSetting::setThreshParams(int _, void* arg) {
	ArucoDetectionSetting* arucoDetectionSetter = static_cast<ArucoDetectionSetting*>(arg);
	assert(arucoDetectionSetter != nullptr);
	if (Settings::adaptiveThreshBlockSize % 2 == 0) Settings::adaptiveThreshBlockSize++;
	if (Settings::adaptiveThreshBlockSize <= 2) Settings::adaptiveThreshBlockSize = 3;
	cv::adaptiveThreshold(arucoDetectionSetter->gray, arucoDetectionSetter->thresholded, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, Settings::adaptiveThreshBlockSize, Settings::adaptiveThreshC);
	cv::findContours(arucoDetectionSetter->thresholded, arucoDetectionSetter->contours, arucoDetectionSetter->contourHierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	arucoDetectionSetter->contoursImg=  cv::Mat(arucoDetectionSetter->img.size(), CV_8UC3, cv::Scalar::all(0));
	for (int i = 0; i < arucoDetectionSetter->contours.size(); i++) {
		cv::drawContours(arucoDetectionSetter->contoursImg, arucoDetectionSetter->contours, i,randColor(),2,2);
	}
	if (arucoDetectionSetter->thresholded.cols > 1000) {
		cv::resize(arucoDetectionSetter->contoursImg, arucoDetectionSetter->contoursImg, cv::Size(1000, 1000.f * arucoDetectionSetter->img.rows / arucoDetectionSetter->img.cols));
	}
	cv::imshow("thresholded", arucoDetectionSetter->thresholded);
	cv::imshow("contoursImg", arucoDetectionSetter->contoursImg);
}





void ArucoDetectionSetting::setPolyParams(int ,void* arg) {
	ArucoDetectionSetting* arucoDetectionSetter = static_cast<ArucoDetectionSetting*>(arg);
	assert(arucoDetectionSetter != nullptr);
	arucoDetectionSetter->candidates.clear();
	cv::Mat polyFilteredImg = cv::Mat(arucoDetectionSetter->img.size(), CV_8UC3, cv::Scalar::all(0));
	int contourIdx = 0;
	for (Poly& contour : arucoDetectionSetter->contours) {
		//剔除輪廓非四邊形或非外輪廓
		if (arucoDetectionSetter->contourIsOuter(arucoDetectionSetter->contourHierarchy, contourIdx)) {
			//剔除輪廓太小者
			if (cv::contourArea(contour) > Settings::minArea) {
				Poly candidate;
				cv::approxPolyDP(contour, candidate, sqrt(contour.size()) * Settings::approxPolyDPEpsilonRatio/100, true);
				//剔除非凸包(任選兩點之連線在點集範圍內)
				if (candidate.size()==4 && cv::isContourConvex(candidate)) {
					//剔除羅擴太靠近照片邊緣
					bool contourTooCloseToBoarder = false;
					for (int i = 0; i < 4; i++) {
						if (candidate[i].x <  Settings::cutBoarder || candidate[i].y <  Settings::cutBoarder || candidate[i].x > arucoDetectionSetter->img.cols  - Settings::cutBoarder || candidate[i].y > arucoDetectionSetter->img.rows  - Settings::cutBoarder) {
							contourTooCloseToBoarder = true;
							break;
						}
					}
					if (!contourTooCloseToBoarder) {
						arucoDetectionSetter->candidates.push_back(candidate);
						cv::drawContours(polyFilteredImg, Polys{ candidate }, -1, randColor(), 6, 6);
					}
				}
			}
		}
		contourIdx++;
	}
	cv::imshow("polyFilteredImg", polyFilteredImg);
}


void ArucoDetectionSetting::setCornerRefineParams(int, void* arg) {
	ArucoDetectionSetting* arucoDetectionSetter = static_cast<ArucoDetectionSetting*>(arg);
	assert(arucoDetectionSetter != nullptr);
	for (const auto& [id, marker] : arucoDetectionSetter->_markers) {
		cv::destroyWindow("cornerRefine" + std::to_string(marker.id));
	}
	arucoDetectionSetter->_markers.clear();
	cv::Mat dispImg;
	arucoDetectionSetter->img.copyTo(dispImg);
	// 原先所有偵測到的標記角點位置
	std::map<MarkerId, Poly> originCorners;
	// 所有標記角點定位優化搜索視窗大小
	std::map<MarkerId, int>winSizes;
	for (Poly& candidate : arucoDetectionSetter->candidates) {
		PolyF candidatef;
		arucoDetectionSetter->makePointsOrder(candidate);
		candidatef.assign(candidate.begin(), candidate.end());
		int winSize = float(Settings::cornerRefineWinsizeRatio) / 100 * sqrt(cv::contourArea(candidate)) / Params::dictionary.bitSize;
		if (winSize == 0) winSize = 1;
		cv::cornerSubPix(arucoDetectionSetter->gray, candidatef, cv::Size(winSize, winSize), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS, 9999999, 0.00000001));
		cv::Mat warped;
		arucoDetectionSetter->warp(candidatef, warped);
		MarkerId id=-1;
		if ((id= arucoDetectionSetter->idMatcher(warped, candidatef))==-1) continue;
		Marker marker(id, Params::dictionary.markerSize(id), candidatef);
		arucoDetectionSetter->_markers.emplace(id,marker);
		winSizes.emplace(id, winSize);
		originCorners.emplace(marker.id, candidate);
	}

	//顯示cornerRefine結果
	for (const auto& [id,marker] : arucoDetectionSetter->_markers) {
		int winSize = winSizes.at(id);
		Poly originCorner = originCorners.at(id);
		/// 展示corner refine後結果以及角點順序是否正確(以字典方向為準，綠點為marker右上角，紅點為右下角，藍點為markery左上角)
		for (int j = 0; j < 4; j++) {
			cv::rectangle(dispImg, cv::Rect(originCorner[j].x-winSize, originCorner[j].y-winSize, winSize * 2, winSize * 2),cv::Scalar(0,0,150));
			switch (j) {
			case 0:
				cv::circle(dispImg, marker.corners.at(j), 5, cv::Scalar(255, 0, 0), 2, cv::MARKER_STAR, 0);
				break;
			case 1:
				cv::circle(dispImg, marker.corners.at(j), 5, cv::Scalar(0, 255, 0), 2, cv::MARKER_CROSS, 0);
				break;
			case 2:
				cv::circle(dispImg, marker.corners.at(j), 5, cv::Scalar(0, 0, 255), 2, cv::MARKER_CROSS, 0);
				break;
			case 3:
				cv::circle(dispImg, marker.corners.at(j), 5, cv::Scalar(100, 100, 100), 2, cv::MARKER_CROSS, 0);
				break;
			}
		}
		cv::Moments m = cv::moments(marker.corners);
		std::vector<cv::Point2i> resampleCorners(4);
		cv::Point2f center2i = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
		//找新的搜索marker視窗(角點距離質心的兩倍距離)
		for (int i = 0; i < 4; i++) resampleCorners[i] = marker.corners[i] * 2 - center2i;
		int minY = resampleCorners[0].y;
		int maxY = resampleCorners[0].y;
		int minX = resampleCorners[0].x;
		int maxX = resampleCorners[0].x;
		for (int i = 1; i < 4; i++) {
			if (resampleCorners[i].y > maxY) maxY = resampleCorners[i].y;
			if (resampleCorners[i].y < minY) minY = resampleCorners[i].y;
			if (resampleCorners[i].x > maxX) maxX = resampleCorners[i].x;
			if (resampleCorners[i].x < minX) minX = resampleCorners[i].x;
		}
		if (minX < 0) minX = 0;
		if (minY < 0) minY = 0;
		if (maxX > arucoDetectionSetter->img.cols) maxX = arucoDetectionSetter->img.cols - 1;
		if (maxY > arucoDetectionSetter->img.rows) maxY = arucoDetectionSetter->img.rows - 1;
		cv::Rect rect(minX, minY, maxX - minX, maxY - minY);
		cv::Mat cut = dispImg(rect);
		cv::namedWindow("cornerRefine" + std::to_string(marker.id), 0);
		cv::imshow("cornerRefine"+std::to_string(marker.id), cut);
	}
	cv::imshow("cornerRefine", dispImg);
}



void ArucoDetectionSetting::save(){
	Settings::save();
}










void ArucoDetectionSetting::run(const Source source,const std::string& sourcePath) {
	Capture capturer(source, sourcePath);
	capturer.run(std::bind(&ArucoDetectionSetting::tune, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3), false);
	cv::destroyAllWindows();
	std::cout << "No further images can be tuned, redo tuning with seen images[y], or exit tuning[n]" << std::endl;
	bool keyWrong = true;
	while (keyWrong) {
		char key;
		std::cin >> key;
		switch (key)
		{
		case 'y':
		case 'Y':
			run(source, sourcePath);
			return;
			break;
		case 'n':
		case 'N':
			return;
			break;
		default:
			std::cout << "key [y] or [n] dumbass" << std::endl;
			break;
		}
	}
}

bool ArucoDetectionSetting::tune(const FrameNumber frameNumber, std::unique_ptr<cv::Mat> img, const std::string& imgName){
	this->img = *img;
	RegionSelect regionSelecter;
	Settings::detectRegions = regionSelecter.run(this->img);
	if (!Settings::detectRegions.empty()) {
		cv::Mat masked;
		createMask(this->img, Settings::detectRegions, masked);
		cv::cvtColor(masked, gray, cv::COLOR_BGR2GRAY);
	}
	else cv::cvtColor(this->img, gray, cv::COLOR_BGR2GRAY);
	if (regionSelecter.stop)return true;
	cv::namedWindow("thresholded",0);
	if (img->cols > 1000) 
		cv::resizeWindow("thresholded", cv::Size(1000, 1000.f * img->rows / img->cols));
	cv::createTrackbar("adaptiveThreshC",  "thresholded", &Settings::adaptiveThreshC, 50, ArucoDetectionSetting::setThreshParams,this);
	cv::createTrackbar("adaptiveThreshBlocksizeRatio",  "thresholded", &Settings::adaptiveThreshBlockSize, 500, ArucoDetectionSetting::setThreshParams, this);
	setThreshParams(0, this);
	cv::namedWindow("polyFilteredImg",0);
	if (img->cols > 1000)
		cv::resizeWindow("polyFilteredImg", cv::Size(1000, 1000.f * img->rows / img->cols));
	cv::createTrackbar("minAreaOfCandidateThresh",  "polyFilteredImg", &Settings::minArea, 10000, ArucoDetectionSetting::setPolyParams, this);
	cv::createTrackbar("approxPolyDPEpsilon",  "polyFilteredImg", &Settings::approxPolyDPEpsilonRatio, 100, ArucoDetectionSetting::setPolyParams, this);
	setPolyParams(0, this);
	cv::namedWindow("cornerRefine",0);
	if (img->cols > 1000)
		cv::resizeWindow("cornerRefine", cv::Size(1000, 1000.f * img->rows / img->cols));
	cv::createTrackbar("cornerRefineWinsizeRatio",  "cornerRefine", &Settings::cornerRefineWinsizeRatio, 100, ArucoDetectionSetting::setCornerRefineParams, this);
	setCornerRefineParams(0, this);
	int key=cv::waitKey(0);
	switch (key) {
	case 27:
		cv::destroyAllWindows();
		return  true;
		break;
	case 115:
	case 83:
		save();
		break;
	case 82:
	case 114:
		Settings::load("settings.json");

	}
	cv::destroyAllWindows();
	return false;
}

Markers ArucoDetectionSetting::markers(){
	Markers rMarkers;
	for (const auto& [id, marker]:_markers) {
		PolyF corners;
		cv::undistortPoints(marker.corners, corners, Params::kmat, Params::distmat, cv::noArray(), Params::kmat);
		rMarkers.emplace(id, Marker(id, marker.size, corners));
	}
	return rMarkers;
}
