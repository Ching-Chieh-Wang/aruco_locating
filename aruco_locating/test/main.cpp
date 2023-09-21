#include "pch.h"
#include "aruco_locating.h"



int main(){
// key: 照片資料夾路徑,value:.json檔的路徑
	ArucoLocating arucoLocator;
	arucoLocator.run(Source::IMAGE, "C:\\Users\\user\\source\\Repos\\Ching-Chieh-Wang\\aruco_crack_sfm\\aruco_crack_sfm\\saved\\26-07-2022 02-32-23\\imgs", true,false, true);
	arucoLocator.save();
}


