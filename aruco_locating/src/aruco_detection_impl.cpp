#include "aruco_detection_impl.h"


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

