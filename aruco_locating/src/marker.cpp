#include "marker.h"






bool Marker::getPoseHistory(cv::Mat& guessRvec, cv::Mat& guessTvec) const{
	auto it = markerTHistories.find(id);
	if (it != markerTHistories.end()) {
		guessRvec = it->second.first;
		guessTvec = it->second.second;
		return true;
	}
	else return false;
}

void Marker::solvePnP(){
	cv::Mat guessRvec, guessTvec;
	if (getPoseHistory(guessRvec, guessTvec)) cv::solvePnPGeneric(markerCoordCorners(), corners, Params::undistKmat, cv::noArray(), _rvecs, _tvecs, true, cv::SOLVEPNP_IPPE_SQUARE, guessRvec, guessTvec, _reprojectionErrors);
	else cv::solvePnPGeneric(markerCoordCorners(), corners, Params::undistKmat, cv::noArray(), _rvecs, _tvecs, true, cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(), _reprojectionErrors); //¥ý«e¤wundistorted
	markerTHistories[id].first = rvec();
	markerTHistories[id].second = tvec();
}

void Marker::show(cv::Mat& img) const{
	cv::putText(img, "ID" + std::to_string(id), corners[0], cv::FONT_HERSHEY_DUPLEX, 2, CV_RGB(255,0,0), 2);
	std::stringstream e, er;
	er.precision(2);
	e.precision(2);
	e << reprojectionError();
	er << reprojectionErrorRatio();
	std::string textPnpResult = "sE:" + e.str() + " ER:" + er.str();
	cv::Scalar color = accessible() ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0);
	cv::putText(img, textPnpResult, corners[0]+cv::Point2f(50,50), cv::FONT_HERSHEY_DUPLEX, 2, color, 2);
}



cv::Point3d Marker::worldCoordCenter() const{
	return cv::Point3d(pose().translation());
}

Clouds Marker::worldCoordCorners() const
{
	Clouds cornersTemp;
	for (const cv::Point3d& corner : markerCoordCorners()) {
		cornersTemp.emplace_back(T() * corner);
	}
	return cornersTemp;
}

Marker::Marker(const MarkerId id, const double& size, const std::vector<cv::Point2f>& corners):id(id),size(size),corners(corners) {
    solvePnP();
}





