#ifndef PARAMS_H
#define PARAMS_H


#include "pch.h""
#include "types.h"
#include "json_reader.h"
#include "dictionary.h"

//常用調整參數
class Params :private JsonReader{
public:
	static std::string filePath;
	//相機內參矩陣
	static cv::Mat kmat;
	//相機扭轉矩陣
	static cv::Mat distmat;
	static Dictionary dictionary;
	static cv::Size imgSize;
	static double markerSize;
	//已知的位姿的marker
	static std::unordered_map<int, double> markers;
	static void load(const std::string& path);
private:
	static std::map<MarkerId, double> getSpecialMarkerSizes(const Json::Value& loader);
};


#endif