#ifndef PARAMS_H
#define PARAMS_H


#include "pch.h""
#include "types.h"
#include "json_reader.h"
#include "dictionary.h"

//�`�νվ�Ѽ�
class Params :private JsonReader{
public:
	static std::string filePath;
	//�۾����ѯx�}
	static cv::Mat kmat;
	//�۾�����x�}
	static cv::Mat distmat;
	static Dictionary dictionary;
	static cv::Size imgSize;
	static double markerSize;
	//�w�����쫺��marker
	static std::unordered_map<int, double> markers;
	static void load(const std::string& path);
private:
	static std::map<MarkerId, double> getSpecialMarkerSizes(const Json::Value& loader);
};


#endif