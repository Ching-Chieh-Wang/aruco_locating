#include <iostream>
#include "pch.h"
#include "src/pba/g2otypes_marker.cpp"
#include "aruco_locating.h"

int main(){
	Front front;
	//front.arucoDetectionSet(Source::IMAGE, "imgs");
	front.run(Source::IMAGE, "imgs",true, false,false);
	front.save("noP");
	return 0;
}


