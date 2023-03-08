//Á{®ÉjsonÅª¨ú¾¹
#include "../pch.h"
#include "params.h"
#include "types.h"
#include "utils.h"
#include "dictionary.h"


void Params::load(const std::string& path) {
	filePath = path;
	Json::Value loader = Json::Value(parse(path));
	kmat = json2cvMat(loader["kmat"]);
	distmat = json2cvMat(loader["distmat"]);
	imgSize = cv::Size(loader["imgSize"][0].asInt(), loader["imgSize"][1].asInt());
	dictionary = Dictionary(Dictionary::DictType(loader["dictionary"].asInt()),loader["markerSize"].asDouble(),getSpecialMarkerSizes(loader));
	std::cout << "load Params successfully" << std::endl;
}

std::map<MarkerId, double> Params::getSpecialMarkerSizes(const Json::Value& loader){
	std::map<MarkerId, double> specialMarkerSizes;
	for (const auto& id : loader["specialMarkerSizes"].getMemberNames()) {
		specialMarkerSizes.emplace(std::stoi(id), loader["specialMarkerSizes"][id].asDouble());
	}
	return specialMarkerSizes;
}


std::string Params::filePath = "";
cv::Mat Params::kmat = cv::Mat();
cv::Mat Params::distmat = cv::Mat();
cv::Size Params::imgSize = cv::Size();
double Params::markerSize = -1;
Dictionary Params::dictionary(Dictionary::DictType::ERROR, -1, std::map<MarkerId, double>{});
std::unordered_map<int, double> Params::markers = {};