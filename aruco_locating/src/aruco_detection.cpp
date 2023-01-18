#include "aruco_detection.h"



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

