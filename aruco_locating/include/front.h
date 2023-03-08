#ifndef FRONT_H
#define FRONT_H
#include "types.h"
#include "analysis.h"

// 前端介面
class Front {
public:
	Front(const std::string& paramsPath="params.json", const std::string& settingsPath="settings.json");
	//調整偵測aruco detection的參數
	void arucoDetectionSet(const Source source, const std::string& sourcePath);
	// 開始跑
	void run(const Source source, const std::string& sourcePath, const bool BA, const bool monitor, const bool parallel=false);
	void save(const std::string path="")const;
	inline void help() const{
		std::cout << "首先透過setSource()選擇影像來源與其路徑\n使用narucoDetectionSet來調整偵測aruco的參數\n沒問題後使用run()開始分析，若要結束分析關掉三維視覺視窗或在視窗上按下exc鍵\n使用save()儲存分析的結果" << std::endl;
	}
private:
	Analysis analyzer;


};
#endif