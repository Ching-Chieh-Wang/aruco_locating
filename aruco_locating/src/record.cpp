#include "pch.h"
#include "record.h"

#include "frame.h"
#include "utils.h"


void Record::addFrame(Frame& frame, const bool saveImg){
	if (saveImg)frame.saveImg();
	else frame.img.reset();
	frames.emplace_back(std::move(frame));

}

void Record::addFrame(Frame& frame, const bool saveImg, const bool isParallel) {
	if (isParallel) {
		if (saveImg) frame.saveImg();
		else frame.img.reset();
		frames.at(frame.frameNumber) = std::move(frame);
	}
	else addFrame(frame, saveImg);
}

void Record::output(const std::string& name) const{
	std::ofstream ofile;
	ofile.open(name+"/Result.csv", std::ios::out, std::ios::trunc);
	ofile << "Frame,ImgName,ID,x,y,z,";
	for (int i = 0; i < 3; i++) ofile << "rvec[" + std::to_string(i) + "],";
	for (int i = 0; i < 3; i++) ofile << "tvec[" + std::to_string(i) + "],";
	ofile  << "error," << std::endl;
	for (const Frame& frame : frames) {
		for (auto& [id, marker] : frame.markers) {
			ofile << frame.frameNumber << ","<<frame.imgName<<',' << id << ',' << marker.worldCoordCenter().x  << "," << marker.worldCoordCenter().y << "," << marker.worldCoordCenter().z << ",";
			cv::Mat rvec, tvec;
			std::tie(rvec, tvec) = T2RvecTvec(marker.pose().inv());
			for (int i = 0; i < 3; i++) ofile << rvec.at<double>(i,0)<<",";
			for (int i = 0; i < 3; i++) ofile << tvec.at<double>(i,0)<<",";
			ofile << marker.reprojectionError() << std::endl;
		}
	}
}

void Record::outputFrames(const std::string& path) const{
	Json::Value savor;
	for (const auto&  frame : frames) {
		Json::Value saveFrame;
		saveFrame["frameNumber"] = frame.frameNumber;
		saveFrame["frameName"] = frame.imgName;
		for (const auto& [id, marker] : frame.markers) {
			Json::Value saveMarker;
			saveMarker["id"] = id;
			saveMarker["size"] = marker.size;
			for (const cv::Point2f& corner : marker.corners) {
				Json::Value saveCorner;
				saveCorner["x"] = corner.x;
				saveCorner["y"] = corner.y;
				saveMarker["corners"].append(saveCorner);
			}
			saveFrame["markers"].append(saveMarker);
		}
		savor["keyFrames"].append(saveFrame);
	}
	Json::StyledWriter sw;
	std::ofstream jsonFile(path + "/map.json", std::ios::out);
	if (!jsonFile.is_open()) {
		throw("Error saving map writing to json");
	}
	jsonFile << sw.write(savor);
	jsonFile.close();
}
