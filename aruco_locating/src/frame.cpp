#include "../pch.h"
#include "frame.h"

#include"types.h"
#include "params.h"


Frame::Frame(const FrameNumber frameNumber,const std::string& imgName, std::unique_ptr<cv::Mat>img, const Markers& markers):frameNumber(frameNumber) {
	this->markers = markers;
	this->imgName = imgName;
	this->img = std::move(img);
}

void Frame::saveImg() {
	//記得要傳值frameNumber以免Frame本身記憶體被釋放就讀不到Frame::frameNumber了
	std::thread saveImgThread(&Frame::saveImgThread, this, frameNumber, std::move(img));
	saveImgThread.detach();
}




void Frame::saveImgThread(FrameNumber frameNumber, std::unique_ptr<cv::Mat> img) {
	std::vector<int> tags = { cv::IMWRITE_TIFF_COMPRESSION, 1 };
	cv::imwrite("imgs_temp/" + std::to_string((int)frameNumber) + ".tiff", *img, tags);
	img.reset();
}

void Frame::show() const {
	cv::Mat dispImg;
	img->copyTo(dispImg);
	for (const auto& [id, marker] : markers) {
		marker.show(dispImg);
	}
	cv::putText(dispImg, imgName, cv::Point2i(50, 50),cv::FONT_HERSHEY_COMPLEX,2,cv::Scalar(100,125,50),4);
	cv::namedWindow("video", 0);
	static int memoryRows = 1500.f * img->rows / img->cols;
	cv::resizeWindow("video", 1500, memoryRows);
	cv::imshow("video", dispImg);
}

std::vector<cv::Point2i> Frame::frameCorners(){
	return Poly{ cv::Point2i(0, 0), cv::Point2i(0, Params::imgSize.height), cv::Point2i(Params::imgSize.width, Params::imgSize.height), cv::Point2i(Params::imgSize.width, 0) };
}
