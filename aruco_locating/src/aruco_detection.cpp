#include "aruco_detection.h"

void ArucoDetection::createMask() {
	if (Settings::detectRegions.empty()) return;
	cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
	cv::Mat masked = cv::Mat::zeros(gray.size(), CV_8UC1);
	cv::fillPoly(mask, Settings::detectRegions, cv::Scalar(255, 255, 255));
	bitwise_and(gray,gray, masked,mask);
	gray = masked;
}


bool ArucoDetection::polyfilter(Poly& candidate, std::vector<cv::Vec4i>& hierarchy, int hierarchyId) {
	//剔除內輪廓
	if (!contourIsOuter(hierarchy, hierarchyId)) return false;
	if (cv::contourArea(candidate) > Settings::minArea) {
		cv::approxPolyDP(candidate, candidate, sqrt(candidate.size()) * Settings::approxPolyDPEpsilonRatio/100, true);
		//篩選形狀類似四邊形者並且為凸包(任選兩點之連線在點集範圍內)
		if (candidate.size() == 4 && cv::isContourConvex(candidate)) {
			//剔除太靠近邊緣者
			for (int i = 0; i < 4; i++) {
				if (candidate[i].x <  Settings::cutBoarder || candidate[i].y <  Settings::cutBoarder || candidate[i].x > thresholded.cols - Settings::cutBoarder || candidate[i].y > thresholded.rows - Settings::cutBoarder) return false;
			}
			return true;
		}
	}
	return false;
}


MarkerId  ArucoDetection::idMatcher(const cv::Mat& warped, PolyF& candidatef) {
	//首先須依照DictionaryBitsize決定wholeBlockSize，+2是因為最外圍兩邊有一預設的黑色區域
	cv::Mat otsued;
	cv::threshold(warped, otsued, 0, 255, cv::THRESH_OTSU);
	int fullsize = Params::dictionary.bitSize + 2;
	//去搜尋在每個Block中有多少白色px，以及每個block中有多少px
	cv::Mat whitePx(fullsize, fullsize, CV_32SC1);
	cv::Mat allPx(fullsize, fullsize, CV_32SC1);
	whitePx.setTo(cv::Scalar::all(0));
	allPx.setTo(cv::Scalar::all(0));
	for (int y = 0; y < otsued.rows; y++) {
		const uchar* ptr = otsued.ptr<uchar>(y);
		int my = float(fullsize) * float(y) / float(otsued.rows);
		for (int x = 0; x < otsued.cols; x++) {
			int mx = float(fullsize) * float(x) / float(otsued.cols);
			if (ptr[x] != 0) whitePx.at<int>(my, mx)++;
			allPx.at<int>(my, mx)++;
		}
	}
	cv::Mat binaryMat = cv::Mat(fullsize, fullsize, CV_8UC1);
	//若每個block的白色px個數大於整個block的px總數之一半，將binaryCode中該block的值設成1，反之為0
	for (int y = 0; y < fullsize; y++)
		for (int x = 0; x < fullsize; x++) {
			if (whitePx.at<int>(y, x) > allPx.at<int>(y, x) / 2) binaryMat.at<uchar>(y, x) = 1;
			else binaryMat.at<uchar>(y, x) = 0;
		}
	//如果發現最外圍不是黑色就略過這個Candidates
	if (!boarderIsBlack(binaryMat, fullsize)) return -1;
	//開始將照片與dictionary進行比對，如果一直match 不到dictionary 的bitCodes，就旋轉arucobinary，直到旋轉4次。
	int rotation = 0;
	int id = -1;
	while (id == -1 && rotation < 4 ) {
		std::bitset<64> arucoBinary;
		int cursor = 0;
		//拿掉最外圍黑色區域，也就是binary code 的精華地帶，binaryCodeMat 存取Mat型式，binaryCode 存取二進位數字
		for (int x = Params::dictionary.bitSize; x > 0; x--)
			for (int y = Params::dictionary.bitSize; y > 0; y--)
				arucoBinary[cursor++] = binaryMat.at<uchar>(x, y);
		id = Params::dictionary.find(arucoBinary);
		if (id != -1) return id;
		else {
			cv::Mat rotatedBinaryMat(fullsize, fullsize, CV_8UC1);
			cv::rotate(binaryMat, binaryMat, cv::ROTATE_90_CLOCKWISE);
			rotation++;
		}
	}
	return -1;
}






void ArucoDetection::cornerRefine(Poly& candidate, PolyF& candidatef) {
	candidatef.assign(candidate.begin(), candidate.end());
	float winsize = float(Settings::cornerRefineWinsizeRatio) / 100 * sqrt(cv::contourArea(candidate)) / Params::dictionary.bitSize;
	cv::cornerSubPix(gray, candidatef, cv::Size(winsize, winsize), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS, 1000000, 0.000001));
}






void ArucoDetection::warp(PolyF& candidate, cv::Mat& warped) {
	std::vector<cv::Point2f> dst_points{ cv::Point2f(0.f,0.f),cv::Point2f(100.f,0.f),cv::Point2f(100.f,100.f),cv::Point2f(0.f,100.f) };
	//進行仿射
	cv::Mat transformed = cv::getPerspectiveTransform(candidate, dst_points);
	int warpSize;
	int candidatesArea = cv::contourArea(candidate);
	if (candidatesArea > 100 * 100) warpSize = 100;
	else warpSize = candidatesArea;
	cv::warpPerspective(gray, warped, transformed, cv::Size(100, 100), cv::INTER_NEAREST);
}

void ArucoDetection::detect(const cv::Mat & image, Markers& markers) {
	cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	createMask();
	Polys candidates;
	std::vector<cv::Vec4i> hierarchy;
	cv::adaptiveThreshold(gray, thresholded, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, Settings::adaptiveThreshBlockSize, Settings::adaptiveThreshC);
	cv::findContours(thresholded, candidates, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
#pragma omp parallel
	{
		std::vector<Marker> markers_private;
	#pragma omp for nowait
		for (int i = 0; i < candidates.size(); i++) {
			Poly* candidate = &candidates.at(i);
			if (!polyfilter(*candidate,hierarchy, i)) continue;
			PolyF candidatef;
			cv::Mat warped;
			std::unique_ptr<Marker> markers;
			makePointsOrder(*candidate);
			cornerRefine(*candidate, candidatef);
			warp(candidatef, warped);
			MarkerId id = -1;
			if ((id=idMatcher(warped, candidatef))==-1) continue;
			Marker marker(id, Params::dictionary.markerSize(id), candidatef);
			cv::undistortPoints(marker.corners, marker.corners, Params::kmat, Params::distmat,cv::noArray(),Params::kmat);
			markers_private.emplace_back(marker);
		}
	#pragma omp critical
		for (const Marker& marker : markers_private) {
			markers.emplace(marker.id, marker);
		}
	}
}

