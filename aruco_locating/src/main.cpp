#include "aruco_locating.h"
#include "types.h"
#include "pch.h""

int main(){
// key: 照片資料夾路徑,value:.json檔的路徑
	ArucoLocating arucoLocator;
	arucoLocator.run(Source::IMAGE, "C:\\Users\\user\\source\\Repos\\Ching-Chieh-Wang\\aruco_crack_sfm\\aruco_crack_sfm\\saved\\11-05-2022 03-07-31\\imgs", true,false, true);
	arucoLocator.save("test");
}


