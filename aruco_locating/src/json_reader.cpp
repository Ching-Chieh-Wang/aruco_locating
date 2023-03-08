#include "../pch.h"
#include "json_reader.h"
#include "utils.h"

Json::Value JsonReader::parse(const std::string jsonPath)  {
	std::ifstream ifs;
	ifs.open(jsonPath);
	Json::Value value;
	Json::Reader reader;
	if (!reader.parse(ifs, value, false))
	{
		throw("reader.parse Error");
	}
	ifs.close();
	return value;
}


//json list Тр cv::Mat
cv::Mat JsonReader::json2cvMat(const Json::Value& jmat)  {
	cv::Mat mat;
	if (jmat[0].type() != Json::ValueType::arrayValue) {
		mat = cv::Mat(1, jmat.size(), CV_64F);
		for (unsigned int i = 0; i < jmat.size(); i++) {
			mat.at<double>(0, i) = jmat[i].asDouble();
		}
	}
	else {
		mat = cv::Mat(jmat[0].size(), jmat.size(), CV_64F);
		for (unsigned int i = 0; i < jmat[0].size(); i++) {
			for (unsigned int j = 0; j < jmat.size(); j++) {
				mat.at<double>(i, j) = jmat[i][j].asDouble();
			}
		}
	}

	return mat;
}

//json list Тр cv::Mat
Json::Value JsonReader::cvMat2Json(const cv::Mat& mat)  {
	Json::Value jsonMat;
	if (mat.cols == 1) {
		for (int i = 0; i < mat.rows; i++) {
			jsonMat.append(mat.at<double>(i, 0));
		}
	}
	else if (mat.rows == 1) {
		for (int i = 0; i < mat.cols; i++) {
			jsonMat.append(mat.at<double>(0, i));
		}
	}
	else {
		for (int i = 0; i < mat.rows; i++) {
			Json::Value jsonRow;
			for (int j = 0; j < mat.cols; j++) {
				jsonRow.append(mat.at<double>(i, j));
			}
			jsonMat.append(jsonRow);
		}
	}

	return jsonMat;
}
Json::Value JsonReader::pose2Json(const std::unordered_map<MarkerId,cv::Affine3d>& markerPoses)  {
	Json::Value jsonMarkersPoses;
	for (const auto& [id, pose] : markerPoses) {
		Json::Value jsonMarkerPoses;
		jsonMarkerPoses["id"] = id;
		cv::Mat rvec, tvec;
		std::tie(rvec, tvec) = T2RvecTvec(pose);
		jsonMarkerPoses["rvec"] = cvMat2Json(rvec);
		jsonMarkerPoses["tvec"] = cvMat2Json(tvec);
		jsonMarkersPoses.append(jsonMarkerPoses);
	}
	return jsonMarkersPoses;
}

std::unordered_map<int, cv::Affine3d> JsonReader::json2Pose(const Json::Value& jmarkers) {
	std::unordered_map<int, cv::Affine3d> markers;
	for (const auto& jmarker : jmarkers) {
		cv::Affine3d pose(json2cvMat(jmarker["rvec"]), (json2cvMat(jmarker["tvec"])));
		markers.emplace(jmarker["id"].asInt(), pose);
	}
	return markers;
}