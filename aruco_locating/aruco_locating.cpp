#include "pch.h"
#include "front.h"


int main(){
// key: 照片資料夾路徑,value:.json檔的路徑
	Front front("params.json", "settings.json");
	//front.arucoDetectionSet(Source::IMAGE, "C:\\Users\\user\\source\\Repos\\Ching-Chieh-Wang\\aruco_crack_sfm\\aruco_crack_sfm\\saved\\redetect\\imgs");
	front.run(Source::IMAGE, "C:\\Users\\user\\source\\Repos\\Ching-Chieh-Wang\\aruco_crack_sfm\\aruco_crack_sfm\\saved\\redetect\\imgs", true,false, true);
	front.saveFrames();
	//front.save();
}


