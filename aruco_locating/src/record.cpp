#include "record.h"



void Record::addFrame(Frame& frame, const bool saveImg){
	frames.emplace_back(std::move(frame));
	if (saveImg)frame.saveImg();
	else frame.img.release();
}

void Record::addFrame(Frame& frame, const bool saveImg, const bool isParallel) {
	if (isParallel) {
		if (saveImg) frame.saveImg();
		else frame.img.release();
		frames.at(frame.frameNumber) = std::move(frame);
	}
	else addFrame(frame, saveImg);
}

void Record::output(const std::string& name) const{
	std::ofstream ofile;
	ofile.open(name+"/Result.csv", std::ios::out, std::ios::trunc);
	ofile << "Frame,ID,x,y,z,";
	for (int i = 0; i < 3; i++) ofile << "rvec[" + std::to_string(i) + "],";
	for (int i = 0; i < 3; i++) ofile << "tvec[" + std::to_string(i) + "],";
	ofile  << "error," << std::endl;
	for (const Frame& frame : frames) {
		for (auto& [id, marker] : frame.markers) {
			ofile << frame.frameNumber << "," << id << ',' << marker.worldCoordCenter().x * 1000 << "," << marker.worldCoordCenter().y * 1000 << "," << marker.worldCoordCenter().z * 1000 << ",";
			cv::Mat rvec, tvec;
			std::tie(rvec, tvec) = T2RvecTvec(marker.pose().inv());
			for (int i = 0; i < 3; i++) ofile << rvec.at<double>(i,0)<<",";
			for (int i = 0; i < 3; i++) ofile << tvec.at<double>(i,0)<<",";
			ofile << marker.reprojectionError() << std::endl;
		}
	}
}
