#include "../pch.h"
#include "../aruco_locating.h"


#include "viz_window.h"
#include "params.h"
#include "settings.h"
#include "aruco_detection_setting.h"





void ArucoLocating::run(const Source source, const std::string& sourcePath, const bool BA , const bool monitor, const bool parallel) {
	std::cout << "proccessing" << std::endl;
	if (!std::filesystem::is_directory("saved") || !std::filesystem::exists("saved")) { // Check if saved folder exists
		std::filesystem::create_directory("saved"); // create saved folder
	}
	if (source == Source::LIVE_CAPTURE) {
		if (!std::filesystem::is_directory("imgs_temp") || !std::filesystem::exists("imgs_temp")) { // Check if imgs_temp folder exists
			std::filesystem::create_directory("imgs_temp"); // create imgs_temp folder
		}
		for (const auto& entry : std::filesystem::directory_iterator("imgs_temp"))
			std::filesystem::remove_all(entry.path()); //remove all contents in imgs_temp folder
	}
	analyzer.run(source,sourcePath,parallel,monitor,BA);
	std::cout << "Finished" << std::endl;
}


void ArucoLocating::saveFrames(std::string path ) const {
	if (path == "") path = savePath;
	std::filesystem::create_directory(path);
	analyzer.outputFrames(path);
	std::cout << "save frames successfully: " + path << std::endl;
}




void ArucoLocating::save( std::string path)  const{
	if (path == "") path=savePath;
	std::filesystem::create_directory(path);
	std::filesystem::create_directory(path + "/imgs");
	analyzer.outputResults(path);
	std::filesystem::copy_file(Params::filePath, path +"/params.json", std::filesystem::copy_options::update_existing);
	std::cout << "save successfully: " + path << std::endl;
}

ArucoLocating::ArucoLocating(const std::string& paramsPath, const std::string& settingsPath) {
	char tt[100];
	time_t now = time(nullptr);
	auto tm_info = localtime(&now);
	strftime(tt, 100, "%Y-%m-%d %H-%M-%S", tm_info);
	savePath = "saved/" + std::string(tt);
	Params::load(paramsPath);
	Settings::load(settingsPath);
}

void ArucoLocating::arucoDetectionSet(const Source source, const std::string& sourcePath){
	ArucoDetectionSetting arucoDetectionSetter;
	arucoDetectionSetter.run(source,sourcePath);
}













