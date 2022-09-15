#include "record.h"



void Record::addFrame(Frame& frame){
	frames.emplace_back(std::move(frame));
}

void Record::addFrame(Frame& frame, const bool isParallel) {
	if (isParallel)	frames.at(frame.frameNumber) = std::move(frame);
	else addFrame(frame);
}

void Record::output(const std::string& name) const{
	std::ofstream ofile;
	ofile.open(name+"/Result.csv", std::ios::out, std::ios::trunc);
	ofile << "Frame" << ',' << "ID" << "," << "x" << "," << "y" << "," << "z" << "," << "unambiguous" << "," << "errorRatio" << "," << "error low" << "," << "error" << "," << std::endl;
	for (const Frame& frame : frames) {
		for (auto& [id, marker] : frame.markers) {
			ofile << frame.frameNumber << "," << id << ',' << marker.worldCoordCenter().x * 1000 << "," << marker.worldCoordCenter().y * 1000 << "," << marker.worldCoordCenter().z * 1000 << ",";
			ofile << marker.unambiguous() ? std::to_string(1) : std::to_string(0);
			ofile << ",";
			ofile << marker.reprojectionErrorRatio();
			ofile << ",";
			ofile << !marker.bigError() ? std::to_string(1) : std::to_string(0);
			ofile << ",";
			ofile << marker.reprojectionError();
			ofile << std::endl;
		}
	}
}
