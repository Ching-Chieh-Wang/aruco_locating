#include "pch.h"
#include "aruco_detection_impl.h"
#include "settings.h"
#include "params.h"


bool ArucoDetectionImpl::boarderIsBlack(cv::Mat& binaryCode, int fullsize) {
	for (int y = 0; y < fullsize; y++)
	{
		int inc = fullsize - 1;
		if (y == 0 || y == fullsize - 1)
			inc = 1;  // for first and last row, check the whole border
		for (int x = 0; x < fullsize; x += inc)
			if (binaryCode.at<uchar>(y, x) != 0)  return false;
	}
	return true;
}

void ArucoDetectionImpl::createMask() {
	if (Settings::detectRegions.empty()) return;
	cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
	cv::Mat masked = cv::Mat::zeros(gray.size(), CV_8UC1);
	cv::fillPoly(mask, Settings::detectRegions, cv::Scalar(255, 255, 255));
	bitwise_and(gray, gray, masked, mask);
	gray = masked;
}


bool ArucoDetectionImpl::polyfilter(Poly& candidate, std::vector<cv::Vec4i>& hierarchy, int hierarchyId) {
	//剔除內輪廓
	if (!contourIsOuter(hierarchy, hierarchyId)) return false;
	if (cv::contourArea(candidate) > Settings::minArea) {
		cv::approxPolyDP(candidate, candidate, sqrt(candidate.size()) * Settings::approxPolyDPEpsilonRatio / 100, true);
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









void ArucoDetectionImpl::cornerRefine(Poly& candidate, PolyF& candidatef) {
	candidatef.assign(candidate.begin(), candidate.end());
	float winsize = float(Settings::cornerRefineWinsizeRatio) / 100 * sqrt(cv::contourArea(candidate)) / Params::dictionary.bitSize;
	cv::cornerSubPix(gray, candidatef, cv::Size(winsize, winsize), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS, 1000000, 0.000001));
}

void ArucoDetectionImpl::makePointsOrder(std::vector<cv::Point2i>& points) {
	int dx1 = points.at(1).x - points.at(0).x;
	int dy1 = points.at(1).y - points.at(0).y;
	int dx2 = points.at(2).x - points.at(0).x;
	int dy2 = points.at(2).y - points.at(0).y;
	int o = (dx1 * dy2) - (dy1 * dx2);
	if (o < 0) {
		std:swap(points.at(1), points.at(3));
	}
	return;
}

bool ArucoDetectionImpl::contourIsOuter(const std::vector<cv::Vec4i>& hierachy, const int idx){
	int checkCount = 0;
	int currentIdx = idx;
	while (hierachy[currentIdx][3] != -1) {
		currentIdx = hierachy[currentIdx][3];
		checkCount++;
	}
	return checkCount % 2 == 0 ? true : false;
}

void ArucoDetectionImpl::createMask(const cv::Mat& img, const Polys& regions, cv::Mat& masked) {
	cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
	cv::fillPoly(mask, regions, cv::Scalar(255, 255, 255));
	bitwise_and(img, img, masked, mask);
}

MarkerId  ArucoDetectionImpl::idMatcher(const cv::Mat& warped, PolyF& candidatef) {
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
	while (id == -1 && rotation < 4) {
		std::bitset<64> arucoBinary;
		int cursor = 0;
		//拿掉最外圍黑色區域，也就是binary code 的精華地帶，binaryCodeMat 存取Mat型式，binaryCode 存取二進位數字
		for (int x = Params::dictionary.bitSize; x > 0; x--)
			for (int y = Params::dictionary.bitSize; y > 0; y--)
				arucoBinary[cursor++] = binaryMat.at<uchar>(x, y);
		id = Params::dictionary.find(arucoBinary);
		if (id != -1) {
			for (int i = 0; i < rotation; i++) {
				candidatef.insert(candidatef.begin(), candidatef.back());
				candidatef.pop_back();
			}
			return id;
		}
		else {
			cv::Mat rotatedBinaryMat(fullsize, fullsize, CV_8UC1);
			cv::rotate(binaryMat, binaryMat, cv::ROTATE_90_CLOCKWISE);
			rotation++;
		}
	}
	return -1;
}

void ArucoDetectionImpl::warp(PolyF& candidate, cv::Mat& warped) {
	std::vector<cv::Point2f> dst_points{ cv::Point2f(0.f,0.f),cv::Point2f(100.f,0.f),cv::Point2f(100.f,100.f),cv::Point2f(0.f,100.f) };
	//進行仿射
	cv::Mat transformed = cv::getPerspectiveTransform(candidate, dst_points);
	int warpSize;
	int candidatesArea = cv::contourArea(candidate);
	if (candidatesArea > 100 * 100) warpSize = 100;
	else warpSize = candidatesArea;
	cv::warpPerspective(gray, warped, transformed, cv::Size(100, 100), cv::INTER_NEAREST);
}

void ArucoDetectionImpl::contrastAdjust(const cv::Mat& img,cv::Mat& adustedImg, const int& contrastFactor){
	adustedImg.create(img.size(), img.type());
#pragma omp parallel for
	for (int i = 0; i < img.rows; i++) {
		const uchar* sptr = img.ptr<uchar>(i);
		uchar* dptr = adustedImg.ptr<uchar>(i);
		for (int j = 0; j < img.cols; j++) {
			uchar b = sptr[img.channels() * j];
			uchar g = sptr[img.channels() * j + 1];
			uchar r = sptr[img.channels() * j + 2];
			dptr[img.channels() * j] = contrastPixelWiseAdjust(b, contrastFactor);
			dptr[img.channels() * j + 1] = contrastPixelWiseAdjust(g, contrastFactor);
			dptr[img.channels() * j + 2] = contrastPixelWiseAdjust(r, contrastFactor);
		}
	}
}

int ArucoDetectionImpl::contrastPixelWiseAdjust(const int& val, const int& contrastFactor){
	float adjustedVal= float(val) / 255;
	float s = 1.f - 3.f * contrastFactor / 400;
	float a1 = 2 * (1 - s);
	float b1 = s;
	float c1 = 0;
	float a2 = 2 * (s - 1);
	float b2 = 4 - 3 * s;
	float c2 = s - 1;
	if (adjustedVal <= 0.5)
		return 255*(a1 * pow(adjustedVal, 2) + b1 * adjustedVal + c1);
	else return 255*(a2 * pow(adjustedVal, 2) + b2 * adjustedVal + c2);
}
