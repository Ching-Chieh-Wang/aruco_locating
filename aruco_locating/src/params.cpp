//Á{®ÉjsonÅª¨ú¾¹

#include "params.h"


void Params::load(const std::string& path) {
	Json::Value loader = Json::Value(parse(path));
	kmat = json2cvMat(loader["kmat"]);
	distmat = json2cvMat(loader["distmat"]);
	imgSize = cv::Size(loader["imgSize"][0].asInt(), loader["imgSize"][1].asInt());
	dictionary = Dictionary(Dictionary::DictType(loader["dictionary"].asInt()),loader["markerSize"].asDouble(),getSpecialMarkerSizes(loader));
	undistKmat = cv::getOptimalNewCameraMatrix(kmat, distmat, imgSize, 1, imgSize, 0);
	cv::initUndistortRectifyMap(kmat, distmat, cv::noArray(), undistKmat, imgSize, CV_32FC2, map1, map2);
}

std::map<MarkerId, double> Params::getSpecialMarkerSizes(const Json::Value& loader){
	std::map<MarkerId, double> specialMarkerSizes;
	for (const auto& id : loader["specialMarkerSizes"].getMemberNames()) {
		specialMarkerSizes.emplace(std::stoi(id), loader["specialMarkerSizes"][id].asDouble());
	}
	return specialMarkerSizes;
}


