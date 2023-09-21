#include "pch.h"
#include "aruco_locating.h"



int main(){
// key: 照片資料夾路徑,value:.json檔的路徑
	ArucoLocating arucoLocator;
	arucoLocator.run(Source::IMAGE, "imgs", true,false, true);
	arucoLocator.save();
}


