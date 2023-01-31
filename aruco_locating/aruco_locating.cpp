#include <iostream>
#include "pch.h"
#include "src/pba/g2otypes_marker.cpp"
#include "aruco_locating.h"


void autoLoadFile(std::map<std::string, std::string>& folderParamsBundles) {
	//for (const auto& fileName : std::filesystem::directory_iterator("D:\\實驗資料處理\\10.18_Test\\南側相機")) {
	//	if (fileName.is_directory()) {
	//		folderParamsBundles.emplace(fileName.path().string(), "D:\\實驗資料處理\\10.18_Test\\南側相機\\params.json");
	//	}
	//}
	folderParamsBundles.emplace("C:\\Users\\KUMA\\Downloads\\123", "D:\\實驗資料處理\\10.18_Test\\南側相機\\params.json");
}

int main(){
// key: 照片資料夾路徑,value:.json檔的路徑
	std::map<std::string,std::string> folderParamsBundles;
	autoLoadFile(folderParamsBundles);
	for (const auto&[ folderPath,paramsPath] : folderParamsBundles) {
		Front front(paramsPath);
		front.arucoDetectionSet(Source::IMAGE, folderPath);
		front.run(Source::IMAGE, folderPath, false, false, true);
		front.save(std::filesystem::path(folderPath).filename().string());
	}
}


