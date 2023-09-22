#include "pch.h"
#include "aruco_locating.h"

int main(){
// key: 照片資料夾路徑,value:.json檔的路徑
	ArucoLocating arucoLocator;
	arucoLocator.arucoDetectionSet(Source::IMAGE, "asset/imgs");
	arucoLocator.run(Source::IMAGE, "asset/imgs", true,true, false);
	arucoLocator.save("asset/saved");
}


