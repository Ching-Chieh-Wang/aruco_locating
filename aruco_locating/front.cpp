#include "front.h"



Front::Front() {
	if (!std::filesystem::is_directory("saved") || !std::filesystem::exists("saved")) { // Check if src folder exists
		std::filesystem::create_directory("saved"); // create src folder
	}
	if (!std::filesystem::is_directory("imgs_temp") || !std::filesystem::exists("imgs_temp")) { // Check if src folder exists
		std::filesystem::create_directory("imgs_temp"); // create src folder
	}
	if (!std::filesystem::is_directory("cracks") || !std::filesystem::exists("cracks")) { // Check if src folder exists
		std::filesystem::create_directory("cracks"); // create src folder
	}
	for (const auto& entry : std::filesystem::directory_iterator("imgs_temp"))
		std::filesystem::remove_all(entry.path());
	for (const auto& entry : std::filesystem::directory_iterator("cracks"))
		std::filesystem::remove_all(entry.path());

};

void Front::run(const Source source,const std::string& path,const bool BA ) {
	_terminate = false;
	Settings::load("settings.json");
	Params::load("params.json");
	VIZWindow::updateWindow();
	VIZWindow::spinOnce(this);
	switch (source) {
	case (Source::LIVE):
		live(path);
		break;
	case (Source::VIDEO):
		video(path);
		break;
	default:
		throw("Video source error. Try LIVE or VIDEO");
	}
#ifdef DEBUG_GET_USEFUL_FRAMES
	extern std::set<FrameNumber> usefulFrames;
	std::cout << "usful frame number:" << std::endl;
	for (const FrameNumber frameNumber : usefulFrames) {
		std::cout << frameNumber << ',';
	}
	std::cout << std::endl;
#endif
	if (globalBA)globalOptmization();
}


void Front::addImg(const int frameNumber, std::unique_ptr<cv::Mat> img) {
	//move 過的東東請勿再引用
	Markers markers= _arucoDetector.run(*img);
	std::shared_ptr<Frame> frame = std::make_shared<Frame>(frameNumber, std::move(markers),std::move(img));
	_tracker.run(*frame,_snap);
	frame->dispImg();
	mapper.lastFrames.emplace_back(std::move(frame));
	if (mapper.lastFrames.size() > 5) mapper.lastFrames.pop_front();
}


void Front::video(const std::string& path) {
	cv::VideoCapture capture(path);
	std::unique_ptr<cv::Mat> img;
	capture.read(*(img = std::make_unique<cv::Mat>()));
	if (true) {
		ArucoDetection detectTemp;
		detectTemp.updateMemory();
		detectTemp.run(*img);
	}
	if (img->cols == 0) {
		throw("cannot load vidoe " + path);
	}
	int frameNumber = 1;
	while(capture.read(*(img = std::make_unique<cv::Mat>()) ) && !_terminate){
		if (!_playOnly.empty()) {
			if (_playOnly.find(frameNumber)==_playOnly.end()) {
				frameNumber++;
				continue;
			}
		}
		addImg(frameNumber++, std::move(img));
		VIZWindow::spinOnce(this);
	}
	capture.release();
	cv::destroyAllWindows();
}

void Front::globalOptmization() {
	std::cout << "Global optimizing" << std::endl;
	if (mapper.keyFrames.empty()) throw("No keyFrames");
	BundleAdjustment::globalOptimization();
}

void Front::reconstruct(const CrackDetectionMethod crackDetectionMethod,const bool alignment, const bool overlapElimination, const bool regionSelect){
	if (mapper.keyFrames.empty()) throw("[align failed] no keyFrame");
	crackReconstructor = ReconstructCrack(crackDetectionMethod, alignment, overlapElimination, regionSelect);
	crackReconstructor.run(clouds);
}


void Front::play(const std::vector<FrameNumber> &_frameNumbers) {
	auto it = _playOnly.begin();
	for (FrameNumber frameNumber : _frameNumbers) {
		it=_playOnly.emplace_hint(it,frameNumber);
	}
}


void Front::live(const std::string& path) {
	cv::VideoCapture capture(std::stoi(path));
	if (!capture.isOpened()) {
		throw("cannot load live cam");
	}
	std::unique_ptr<cv::Mat> img;
	int frameNumber = 1;
	capture.read(*(img = std::make_unique<cv::Mat>()));
	if (true) {
		while (img->size() != Params::imgSize) {  //讀取失敗
			std::cout << "load img error" << std::endl;
		}
		ArucoDetection detectTemp;
		detectTemp.updateMemory();
		detectTemp.run(*img);
	}
	while (!_terminate && !VIZWindow::wasStopped() ) {
		capture.read(*(img=std::make_unique<cv::Mat>()));
		while (img->size()!= Params::imgSize) {  //讀取失敗
			std::cout << "load img error" << std::endl;
			capture.release();  //重開
			capture.open(std::stoi(path));
			capture.read(*(img = std::make_unique<cv::Mat>()));
		}
		addImg(frameNumber++, std::move(img));
		VIZWindow::spinOnce(this);
	}
	capture.release();
	cv::destroyAllWindows();
}




void Front::save( std::string path)  {
	if (path != "" || _filePath=="") {
		if (path != "") _filePath = "saved/"+path;
		else {
			const auto now = std::chrono::system_clock::now();
			_filePath = "saved/" + std::format("{:%d-%m-%Y %H-%M-%OS}", now);
		}
		std::filesystem::create_directory(_filePath);
		std::filesystem::create_directory(_filePath + "/imgs");
		Json::Value savor;
		for (const auto& [frameNumber, keyFrame] : mapper.keyFrames) {
			Json::Value saveKeyFrame;
			saveKeyFrame["frameNumber"] = frameNumber;
			saveKeyFrame["snapped"] = keyFrame.snapped;
			cv::Mat rvec, tvec;
			std::tie(rvec, tvec) = T2RvecTvec(keyFrame.pose());
			saveKeyFrame["pose"]["rvec"] = cvMat2Json(rvec);
			saveKeyFrame["pose"]["tvec"] = cvMat2Json(tvec);
			saveKeyFrame["vertexId"] = keyFrame.vertexID();
			for (const auto& [id, marker] : keyFrame.markers) {
				Json::Value saveMarker;
				saveMarker["id"] = id;
				saveMarker["size"] = marker->size;
				saveMarker["markerType"] = marker->markerType;
				for (const cv::Point2f& corner : marker->corners) {
					Json::Value saveCorner;
					saveCorner["x"] = corner.x;
					saveCorner["y"] = corner.y;
					saveMarker["corners"].append(saveCorner);
				}
				saveKeyFrame["markers"].append(saveMarker);
			}
			savor["keyFrames"].append(saveKeyFrame);
		}
		for (const auto& [id, keyMarker] : mapper.keyMarkers) {
			Json::Value saveKeyMarker;
			cv::Mat rvec, tvec;
			std::tie(rvec, tvec) = T2RvecTvec(keyMarker.pose());
			saveKeyMarker["id"] = id;
			saveKeyMarker["pose"]["rvec"] = cvMat2Json(rvec);
			saveKeyMarker["pose"]["tvec"] = cvMat2Json(tvec);
			saveKeyMarker["vertexId"] = keyMarker.vertexID();
			saveKeyMarker["size"] = keyMarker.size;
			for (const auto& [frameNumber, _] : keyMarker.shownKeyFrames) {
				saveKeyMarker["shownKeyFrameNumbers"].append(frameNumber);
			}
			savor["keyMarkers"].append(saveKeyMarker);
		}
		for (const auto& [id, _] : mapper.initKeyMarkersPtr) {
			savor["initKeyMarkerIds"].append(id);
		}
		savor["currentVertexId"] = MapObj::currentVertexId;
		Json::StyledWriter sw;
		std::ofstream jsonFile(_filePath+ "/map.json", std::ios::out);
		if (!jsonFile.is_open()) {
			throw("Error saving map writing to json");
		}
		jsonFile << sw.write(savor);
		jsonFile.close();
		Params::save(_filePath + "/params.json");
		Settings::save(_filePath + "/settings.json");
		std::filesystem::copy("imgs_temp", _filePath +"/imgs", std::filesystem::copy_options::update_existing);
	}
	if(!clouds.empty()) cv::viz::writeCloud(_filePath + "/clouds.ply", clouds);

}


void Front::load(const std::string& path) {
	_filePath = "saved/" + path;
	std::filesystem::copy(_filePath + "/imgs", "imgs_temp", std::filesystem::copy_options::update_existing);
	Json::Value savedMap = parse(_filePath + "/map.json");
	mapper.keyFrames.clear();
	mapper.keyMarkers.clear();
	Params::load(_filePath + "/params.json");
	Settings::load(_filePath + "/settings.json");
	std::map<MarkerId, std::vector<MarkerId>> keyMarkerShonwKeyFrames;
	for (const auto& savedKeyMarker : savedMap["keyMarkers"]) {
		cv::Affine3d pose = rvecTvec2T(json2cvMat(savedKeyMarker["pose"]["rvec"]), json2cvMat(savedKeyMarker["pose"]["tvec"]));
		MarkerId id = savedKeyMarker["id"].asInt();
		KeyMarker keyMarker(id, pose, savedKeyMarker["size"].asDouble(), savedKeyMarker["vertexId"].asInt());
		for (const auto& frameNumber : savedKeyMarker["shownKeyFrameNumbers"]) {
			keyMarkerShonwKeyFrames[id].emplace_back( frameNumber.asInt());
		}
		mapper.addKeyMarker(keyMarker);
	}
	for (const auto& savedKeyFrame : savedMap["keyFrames"]) {
		Markers markers;
		for (const auto& savedMarker : savedKeyFrame["markers"]) {
			MarkerId id = savedMarker["id"].asInt();
			auto it = mapper.keyMarkers.find(id);
			std::vector<cv::Point2f> corners;
			corners.reserve(4);
			for (const auto& corner : savedMarker["corners"]) {
				corners.emplace_back(cv::Point2f(corner["x"].asFloat(), corner["y"].asFloat()));
			}
			markers.emplace(id, std::make_shared<Marker>(id, savedMarker["size"].asDouble(),corners, Marker::MarkerType(savedMarker["markerType"].asInt())));
		}
		cv::Affine3d pose = rvecTvec2T(json2cvMat(savedKeyFrame["pose"]["rvec"]), json2cvMat(savedKeyFrame["pose"]["tvec"]));
		FrameNumber frameNumber = savedKeyFrame["frameNumber"].asInt();
		KeyFrame keyFrame(frameNumber, std::move(markers), pose, savedKeyFrame["snapped"].asBool(),savedKeyFrame["vertexId"].asInt());
		mapper.addKeyFrame(keyFrame);
	}
	for (const auto& initMarkerIds : savedMap["initMarkerIds"]) {
		MarkerId id = initMarkerIds.asInt();
		mapper.initKeyMarkersPtr.emplace(id, &mapper.keyMarkers.at(id));
	}
	for (auto& [id, shownKeyFrameNumbers] : keyMarkerShonwKeyFrames) {
		std::unordered_map<FrameNumber, KeyFrame*> shownKeyFrames;
		for (const int frameNumber : shownKeyFrameNumbers) {
			shownKeyFrames.emplace(frameNumber, &mapper.keyFrames.at(frameNumber));
		}
		mapper.keyMarkers.at(id).shownKeyFrames = shownKeyFrames;
	}
	MapObj::currentVertexId = savedMap["currentVertexId"].asInt();
	VIZWindow::updateWindow();
	VIZWindow::spinOnce(this);
}

void Front::measure(const std::string& imageName){
	if (clouds.empty()) throw("reconstruct cloud first");
	cv::Affine3d projectedPlane = crackReconstructor._projectPlanes.at(std::stoi(imageName)).translate(cv::Vec3d(0, 0, -Settings::thickness));
	distanceMeasurer.run(imageName, projectedPlane);
}



void Front::adjustSetting(){
	char option=-1;
	std::cout << "set what?  [1]Arucodetection,[2] Pose Estimation,[3] Sample ,[4]Tracking" << std::endl;
	std::cin >> option;
	try {
		switch (option){
		case '1': {
			char option2 = -1;
			std::string value = "";
			std::cout << "set what?  [1]resizeCols,[2] cutBoarder, [3]adaptiveThreshC,[4]adaptiveThreshBlocksizeRatio, [5]minAreaOfCandidateRatioThresh,[6]approxPolyDPEpsilon,[7]cornerRefineWinsizeRatio" << endl;
			std::cin >> option2;
			switch (option2) {
			case '1': {
				bool stop = false;
				while (!stop) {
					std::cout << "default resizeCols: " << Settings::resizeCols << endl;
					std::cout << "change to?" << std::endl;
					std::cin >> value;
					Settings::resizeCols = std::stoi(value);
					std::cout << "press y to stop setting" << endl;
					char stopPress;
					std::cin >> stopPress;
					if (stopPress == 'y')stop = true;
				}
				break;
			}
				
			case '2': {
				bool stop = false;
				while (!stop) {
					std::cout << "default cutBoarder: " << Settings::cutBoarder << endl;
					std::cout << "change to?" << std::endl;
					std::cin >> value;
					Settings::cutBoarder = std::stof(value);
					std::cout << "press y to stop setting" << endl;
					char stopPress;
					std::cin >> stopPress;
					if (stopPress == 'y')stop = true;
				}
				break;
			}
				
			case '3':{
				bool stop = false;
				while (!stop) {
					std::cout << "default adaptiveThreshC: " << Settings::adaptiveThreshC << endl;
					std::cout << "change to?" << std::endl;
					std::cin >> value;
					Settings::adaptiveThreshC = std::stoi(value);
					std::cout << "press y to stop setting" << endl;
					char stopPress;
					std::cin >> stopPress;
					if (stopPress == 'y')stop = true;
				}
				break;
			}
				
			case '4': {
				bool stop = false;
				while (!stop) {
					std::cout << "default adaptiveThreshBlocksizeRatio: " << Settings::adaptiveThreshBlocksizeRatio << endl;
					std::cout << "change to?" << std::endl;
					std::cin >> value;
					Settings::adaptiveThreshBlocksizeRatio = std::stof(value);
					std::cout << "press y to stop setting" << endl;
					char stopPress;
					std::cin >> stopPress;
					if (stopPress == 'y')stop = true;
				}
				break;
			}
			case '5': {
				bool stop = false;
				while (!stop) {
					std::cout << "default minAreaOfCandidateRatioThresh: " << Settings::minAreaOfCandidateRatioThresh << endl;
					std::cout << "change to?" << std::endl;
					std::cin >> value;
					Settings::minAreaOfCandidateRatioThresh = std::stof(value);
					std::cout << "press y to stop setting" << endl;
					char stopPress;
					std::cin >> stopPress;
					if (stopPress == 'y')stop = true;
				}
				break;
			}
				
			case '6': {
				bool stop = false;
				while (!stop) {
					std::cout << "default approxPolyDPEpsilon: " << Settings::approxPolyDPEpsilon << endl;
					std::cout << "change to?" << std::endl;
					std::cin >> value;
					Settings::approxPolyDPEpsilon = std::stof(value);
					std::cout << "press y to stop setting" << endl;
					char stopPress;
					std::cin >> stopPress;
					if (stopPress == 'y')stop = true;
				}
				break;
			}
				
			case '7': {
				bool stop = false;
				while (!stop) {
					std::cout << "default cornerRefineWinsizeRatio: " << Settings::cornerRefineWinsizeRatio << endl;
					std::cout << "change to?" << std::endl;
					std::cin >> value;
					Settings::cornerRefineWinsizeRatio = std::stof(value);
					std::cout << "press y to stop setting" << endl;
					char stopPress;
					std::cin >> stopPress;
					if (stopPress == 'y')stop = true;
				}
				break;
			}
			default:
				std::cout<<"wrong input";
			}
			updateMemory();
			break;
		};
		case '2': {
			char option2 = -1;
			std::string value = "";
			std::cout << "set what?  [1]unambiguousErrorRaioThresh,[2] projectionErrorThresh" << std::endl;
			std::cin >> option2;
			switch (option2){
			case '1':
				std::cout << "default unambiguousErrorRaioThresh: " << Settings::unambiguousErrorRaioThresh << std::endl;
				std::cout << "change to?" << std::endl;
				std::cin >> value;
				Settings::unambiguousErrorRaioThresh = std::stof(value);
				break;
			case '2':
				std::cout << "default projectionErrorThresh: " << Settings::projectionErrorThresh << std::endl;
				std::cout << "change to?" << std::endl;
				std::cin >> value;
				Settings::projectionErrorThresh = std::stof(value);
				break;
			default:
				std::cout<< "wrong input";
			}
			break;
		}
		case '3': {
			char option2 = -1;
			std::string value = "";
			std::cout << "set what?  [1]nonPerspectiveAngle,[2] perspectiveAngle" << std::endl;
			std::cin >> option2;
			switch (option2) {
			case '1':
				std::cout << "default nonPerspectiveAngle: " << Settings::nonPerspectiveAngle << std::endl;
				std::cout << "change to?" << std::endl;
				std::cin >> value;
				Settings::nonPerspectiveAngle = std::stof(value);
				break;
			case '2':
				std::cout << "default perspectiveAngle: " << Settings::perspectiveAngle << std::endl;
				std::cout << "change to?" << std::endl;
				std::cin >> value;
				Settings::perspectiveAngle = std::stof(value);
				break;
			default:
				std::cout<<"wrong input";;
			}
			break;

		}
		case '4': {
			char option2 = -1;
			std::string value = "";
			std::cout << "set what?  [1]minRefMarker" << std::endl;
			std::cin >> option2;
			switch (option2) {
			case '1':
				std::cout << "default minRefMarker: " << Settings::minSmallErrMarkerCount << std::endl;
				std::cout << "change to?" << std::endl;
				std::cin >> value;
				Settings::minSmallErrMarkerCount = std::stoi(value);
				break;
			default:
				std::cout << "wrong input";;
			}
			break;

		}
		default:
			std::cout<<("wrong input");
			break;;
		}
		std::cout << "change succesfully" << std::endl;
	}catch(const std::exception& e) {
		std::cout << e.what() << std::endl;
		std::cout << "change aborted" << endl;
	}
	
}

void Front::testMarkerPoseError(const std::map<MarkerId, cv::Affine3d>& testPoses){
	for (const auto& [id, pose] : testPoses) {
		auto it = mapper.keyMarkers.find(id);
		if (it == mapper.keyMarkers.end()) {
			std::cout << "No such marker id in map:" << id << std::endl;
		}
		else {
			std::cout << "ID " << id << "error:";
			double lieError = 0;
			double error = 0;
			for (int i = 0; i < 4; i++) {
				error += cv::norm(pose.inv() * it->second.markerCoordCorners().at(i)-it->second.pose()*it->second.markerCoordCorners().at(i))*1000;
			}
			std::cout << error << std::endl;
			std::cout << "Lie error:" << (affineToSE3(pose).log() - affineToSE3(it->second.pose()).log()).squaredNorm()<< std::endl;
		}
	}
}



Front::~Front() {
	std::filesystem::remove_all("saved_imgs");
	std::filesystem::remove_all("crack_imgs");
}


void VIZWindow::spinOnce(Front* front) {
	while (freezeViz) {
		_window.registerKeyboardCallback(keyboardViz3d, &_window);
		_window.spinOnce(1, true);
	}
	std::pair<cv::viz::Viz3d*, Front*> pack(&_window, front);
	_window.registerKeyboardCallback(keyboardViz3d, &pack);
	_window.spinOnce(1, true);
}

void  VIZWindow::keyboardViz3d(const cv::viz::KeyboardEvent& w, void* t){
	std::pair<cv::viz::Viz3d*, Front*>* pair = static_cast<std::pair<cv::viz::Viz3d*,Front*>*>(t);
	Front* front = pair->second;
	if (w.action) {
		switch (w.code) {
		case 'f':
			freezeViz = !freezeViz;
			break;
		case 'b':
			Settings::dispBA = !Settings::dispBA;
			break;
		case ' ':
			front->snap();
			break;
		case '\x1b':
			front->terminate()=true;
			break;
		case 's':
			front->adjustSetting();
			break;
		}
	}
}

