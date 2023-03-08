#ifndef FRAME_H
#define FRAME_H

#include "marker.h"
class Frame {
private:
	void saveImgThread(FrameNumber frameNumber, std::unique_ptr<cv::Mat> img);

public:
	Frame() {}
	Frame(const FrameNumber frameNumber,const std::string& imgName, std::unique_ptr<cv::Mat> img, const Markers& markers);
	void saveImg();
	void show()const;
	static std::vector<cv::Point2i> frameCorners();
	std::string imgName="";
	FrameNumber frameNumber = -1;
	std::unique_ptr<cv::Mat> img;
	Markers markers;

};
#endif