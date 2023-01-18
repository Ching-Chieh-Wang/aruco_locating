#include "viz_window.h"


void VIZWindow::updateWindow(const Frame& frame){
	_window.removeAllWidgets();
	cv::viz::WCameraPosition coord(1);
	_window.showWidget("coord", coord);
	cv::viz::WText imgName(frame.imgName,cv::Point2i(20,20));
	_window.showWidget("ImgName", imgName);
	for (const auto& [id, marker] : frame.markers) {
		cv::viz::WPlane plane(cv::Size2d(marker.size,marker.size), cv::viz::Color::black());
		_window.showWidget("Marker" + std::to_string(marker.id), plane,marker.T());
		cv::viz::WCameraPosition markerPose(marker.size);
		_window.showWidget("MarkerPose" + std::to_string(marker.id), markerPose, marker.T());
		cv::viz::WText3D markerID(std::to_string(marker.id), cv::Vec3d(0, 0, 0), marker.size);
		_window.showWidget("ID" + std::to_string(marker.id), markerID,marker.T());
	}
	show();
}

void VIZWindow::show(){
	_window.registerKeyboardCallback(keyboardViz3d, this);
	_window.spinOnce();
}

VIZWindow::~VIZWindow(){
	_window.close();
}





void  VIZWindow::keyboardViz3d(const cv::viz::KeyboardEvent& w, void* t) {
	VIZWindow* vIZWindow = static_cast<VIZWindow*>(t);
	if (w.action) {
		switch (w.code) {
		case 'f':
			vIZWindow->isFreezeViz = !vIZWindow->isFreezeViz;
			break;
		case 'b':
			Settings::dispBA = !Settings::dispBA;
			break;
		case ' ':
			vIZWindow->isSnap=true;
			break;
		case '\x1b':
			vIZWindow->isTerminate = true;
			break;
		case 'a':
			vIZWindow->isAdjustSetting=true;
			break;
		case 'r':
			vIZWindow->isRescue = true;
			break;
		}
	}
}