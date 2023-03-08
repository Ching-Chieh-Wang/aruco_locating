#include "pch.h"
#include "front.h"

void autoLoadFile(std::map<std::string, std::pair<std::string,std::string>>& folderParamsSettingsBundles) {
	folderParamsSettingsBundles.emplace("datum", std::pair <std::string, std::string>("params.json", "settings.json"));
}

int main(){
// key: 照片資料夾路徑,value:.json檔的路徑
	std::map<std::string, std::pair<std::string, std::string>> folderParamsBundles;
	autoLoadFile(folderParamsBundles);
	for (const auto&[ folderPath,paramsSettingsPath] : folderParamsBundles) {
		Front front(paramsSettingsPath.first, paramsSettingsPath.second);
		/*front.arucoDetectionSet(Source::IMAGE, folderPath);*/
		front.run(Source::IMAGE, folderPath, true, true, false);
		front.save(std::filesystem::path(folderPath).filename().string());
	}
}


