#ifndef CAPTURE_H
#define CAPTURE_H


#include "types.h"

class Capture {
public:
	int totalFramesCount = -1;
	Capture(const Source source, const std::string& sourcePath);
	void run(const std::function<bool(const FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func, bool parallel);
private:
	Source _sourceType=Source::ERROR;
	std::string _sourcePath="";
	// 影像來源從特定資料夾中的照片
	void image(const std::string& sourcePath, const std::function<bool(FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func, const bool isParallel);
	// 影像來源從影片
	void video(const std::function<bool(FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func, const bool isParallel);
	// 影像來源從liveCam中手動擷取的照片
	void liveCapture(const std::function<bool(FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func);

};

#endif