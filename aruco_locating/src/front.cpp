#include "front.h"
#include "viz_window.h"






void Front::run(const Source source, const std::string& sourcePath, const bool BA , const bool monitor, const bool parallel) {
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
}





void Front::save(const std::string path)  const{

	std::string savePath = "";
	if (path != "") savePath = "saved/"+path;
	else {
		char tt[100];
		time_t now = time(nullptr);
		auto tm_info = localtime(&now);
		strftime(tt, 100, "%Y-%m-%d %H-%M-%S", tm_info);
		savePath = "saved/"+std::string(tt);
	}
	std::filesystem::create_directory(savePath);
	std::filesystem::create_directory(savePath + "/imgs");
	analyzer.outputResults(savePath);
}

Front::Front(){
	Params::load("params.json");
	Settings::load("settings.json");
}

void Front::arucoDetectionSet(const Source source, const std::string& sourcePath){
	ArucoDetectionSetting arucoDetectionSetter;
	arucoDetectionSetter.run(source,sourcePath);
}













