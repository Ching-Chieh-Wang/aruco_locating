#include "pch.h"
#include "aruco_detection.h"
#include "types.h"
#include "marker.h"
#include "params.h"


void ArucoDetection::detect(const cv::Mat & image, Markers& markers) {
	cv::Mat adjustedImg;
	if (Settings::contrastFactor != 0) contrastAdjust(image, adjustedImg, Settings::contrastFactor);
	else adjustedImg = image;
	cv::cvtColor(adjustedImg, gray, cv::COLOR_BGR2GRAY);
	createMask();
	Polys candidates;
	std::vector<cv::Vec4i> hierarchy;
	cv::adaptiveThreshold(gray, thresholded, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, Settings::adaptiveThreshBlockSize, Settings::adaptiveThreshC);
	cv::findContours(thresholded, candidates, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	markers.clear();
	int i = 0;
	for (auto& candidate:candidates) {
		if (!polyfilter(candidate,hierarchy, i++)) continue;
		PolyF candidatef;
		cv::Mat warped;
		makePointsOrder(candidate);
		cornerRefine(candidate, candidatef);
		warp(candidatef, warped);
		MarkerId id = -1;
		if ((id=idMatcher(warped, candidatef))==-1) continue;
		Marker marker(id, Params::dictionary.markerSize(id), candidatef);
		cv::undistortPoints(marker.corners, marker.corners, Params::kmat, Params::distmat,cv::noArray(),Params::kmat);
		markers.emplace(marker.id, marker);
	}
}

