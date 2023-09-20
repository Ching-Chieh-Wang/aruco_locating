#include "../aruco_locating.h"


int main(){
// key: 照片資料夾路徑,value:.json檔的路徑
	ArucoLocating arucoLocator;
	arucoLocator.run(Source::IMAGE, "C:\\Users\\user\\Desktop\\i9\\論文重建\\imgs", true,false, true);
	arucoLocator.save();
}


