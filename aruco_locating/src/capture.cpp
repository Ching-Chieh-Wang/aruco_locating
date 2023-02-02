#include "capture.h"


void Capture::run(const std::function<bool(const FrameNumber,std::unique_ptr<cv::Mat>,const std::string&)>& func,bool parallel){
	switch (_sourceType) {
		case Source::LIVE_CAPTURE: {
			cv::VideoCapture capturer(std::stoi(_sourcePath));
			std::unique_ptr<cv::Mat> img;
			FrameNumber frameNumber = 0;
			bool stop = false;
			while (!stop) {
				while (!capturer.isOpened() || img->size() != Params::imgSize) {  //讀取失敗
					std::cout << "Cannot load live image, check your configuration and connection" << std::endl;
					capturer.release();  //重開
					capturer.open(std::stoi(_sourcePath));
					capturer.read(*img);
				}
				std::unique_ptr<cv::Mat> img;
				capturer.read(*(img = std::make_unique<cv::Mat>()));
				stop=func(frameNumber, std::move(img),std::to_string(frameNumber));
				frameNumber++;
			}
			capturer.release();
			break;
		}
		case Source::IMAGE: {
			std::vector<std::string> imgPaths;
			cv::glob(_sourcePath, imgPaths);
			if (parallel) {
#pragma omp parallel for
				for (FrameNumber frameNumber = 0; frameNumber < imgPaths.size(); frameNumber++) {
					func(frameNumber, std::make_unique<cv::Mat>(cv::imread(imgPaths.at(frameNumber))), std::filesystem::path(imgPaths.at(frameNumber)).filename().string());
				}
			}
			else {
				FrameNumber frameNumber = 0;
				for (const std::string& path : imgPaths) {
					bool stop=func(frameNumber++, std::make_unique<cv::Mat>(cv::imread(path)), std::filesystem::path(imgPaths.at(frameNumber)).filename().string());
					if (stop) break;
				}
			}

		}
	}
}


Capture::Capture(const Source source, const std::string& sourcePath){
	cv::VideoCapture capturer;
	_sourceType = source;
	_sourcePath = sourcePath;
	switch (source) {
	case Source::LIVE_CAPTURE: {
		throw("LIVE_CAPTURE currently not supported");
		try {
			std::stoi(_sourcePath);
		}
		catch (...) {
			throw "If source is live_capture sourcePath must be int,such as 0 for default usb camera";
		}
		cv::Mat img;
		capturer.open(std::stoi(sourcePath));
		capturer.read(img);
		{
			while (!capturer.isOpened() || img.size() != Params::imgSize) {  //讀取失敗
				std::cout << "Cannot load live image, check your configuration and connection" << std::endl;
				capturer.release();  //重開
				capturer.open(std::stoi(_sourcePath));
				capturer.read(img);
			}
		}
		capturer.release();
		break;
	}
	case Source::VIDEO: {
		throw("VIDEO currently not supported");
		cv::VideoCapture capturer;
		cv::Mat img;
		capturer.open(sourcePath);
		if(!capturer.read(img)) throw("Cannot load video\""+sourcePath+"\"");
		totalFramesCount = capturer.get(cv::CAP_PROP_FRAME_COUNT);
		capturer.release();
		break;
	}
		
	case Source::IMAGE: {
		std::vector<std::string> imgPaths;
		cv::glob(sourcePath, imgPaths);
		if (imgPaths.size() == 0) throw("Cannot find images in path \"" + sourcePath + "\"");
		totalFramesCount = imgPaths.size();
		break;
	}
	default:
		throw("Capture source error");
	}
}

void Capture::image(const std::string& sourcePath,const std::function<bool(FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func, const bool isParallel) {
	std::vector<std::string> fileNames;
	cv::glob(sourcePath, fileNames);
	if (fileNames.empty()) throw("No such path or no file in \"" + sourcePath + "\"");
	if (isParallel) {
#pragma omp parallel for
		for (int frameNumber = 0; frameNumber < fileNames.size(); frameNumber++) {
			std::string fileName = fileNames.at(frameNumber);
			std::unique_ptr<cv::Mat> img = std::make_unique<cv::Mat>(cv::imread(fileName));
			func(frameNumber, std::move(img),fileName);
		}
	}
	else {
		FrameNumber frameNumber = 0;
		for (const std::string& fileName : fileNames) {
			std::unique_ptr<cv::Mat> img = std::make_unique<cv::Mat>(cv::imread(fileName));
			func(frameNumber++, std::move(img), fileName);
		}
	}
}

void Capture::video(const std::function<bool(FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func, const bool isParallell) {
	cv::VideoCapture capturer;
	std::unique_ptr<cv::Mat> img;
	if (isParallell) {
#pragma omp parallel for
		for (int frameNumber = 0; frameNumber < totalFramesCount; frameNumber++) {
			cv::VideoCapture capture(_sourcePath);
			capture.set(cv::CAP_PROP_POS_FRAMES, frameNumber);
			std::unique_ptr<cv::Mat> img;
			capturer.read(*(img = std::make_unique<cv::Mat>()));
			func(frameNumber, std::move(img),std::to_string(frameNumber));
			capture.release();
		}
	}
	else {
		bool stop = false;
		while (capturer.read(*(img = std::make_unique<cv::Mat>())) && func(capturer.get(cv::CAP_PROP_POS_FRAMES), std::move(img),std::to_string(capturer.get(cv::CAP_PROP_POS_FRAMES)))) {}
		capturer.release();
	}
}

void Capture::liveCapture(const std::function<bool(FrameNumber, std::unique_ptr<cv::Mat>, const std::string&)>& func) {
	cv::VideoCapture capturer;
	bool stop = false;
	std::unique_ptr<cv::Mat> img;
	capturer.set(cv::CAP_PROP_POS_FRAMES, std::stoi(_sourcePath));
	FrameNumber frameNumber=0;
	while (!stop) {
		capturer.read(*(img = std::make_unique<cv::Mat>()));
		while (!capturer.isOpened() || img->size() != Params::imgSize) {  //讀取失敗
			std::cout << "Cannot load live image, check your configuration and connection" << std::endl;
			capturer.release();  //重開
			capturer.open(std::stoi(_sourcePath));
			capturer.read(*img);
		}
		stop=func(frameNumber, std::move(img),std::to_string(frameNumber));
		frameNumber++;
	}
	capturer.release();
	cv::destroyAllWindows();
}