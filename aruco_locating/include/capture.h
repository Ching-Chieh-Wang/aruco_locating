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
	// �v���ӷ��q�S�w��Ƨ������Ӥ�
	void image(const std::string& sourcePath, const std::function<bool(FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func, const bool isParallel);
	// �v���ӷ��q�v��
	void video(const std::function<bool(FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func, const bool isParallel);
	// �v���ӷ��qliveCam������^�����Ӥ�
	void liveCapture(const std::function<bool(FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func);

};

#endif