#ifndef JSON_READER
#define JSON_READER

#include "pch.h"
#include "types.h"

class JsonReader {
protected:
	//json�ɮ��^����
	static Json::Value parse(const std::string jsonPath);


	//json list �� cv::Mat
	static cv::Mat json2cvMat(const Json::Value& jmat);
	//json list �� cv::Mat
	static Json::Value cvMat2Json(const cv::Mat& mat);
	static Json::Value pose2Json(const std::unordered_map<MarkerId,cv::Affine3d>& markerPoses);
	static std::unordered_map<int, cv::Affine3d> json2Pose(const Json::Value& jmarkers);
};
#endif // !JSON_READER
