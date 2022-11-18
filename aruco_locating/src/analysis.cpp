#include "analysis.h"



void Analysis::run(const Source source, const std::string& sourcePath, const bool isParallel, const bool isMonitor,bool isBA) {
	_source = source;
	_isBA = isBA;
	_isParallel = isParallel;
	_isMonitor = isMonitor;
	Capture capture(source,sourcePath);
	if (isParallel) {
		_isParallel = true;
		recorder.frames.resize(capture.totalFramesCount);
	}
	capture.run(std::bind(&Analysis::analysis, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3), isParallel);
}

bool Analysis::analysis(const FrameNumber frameNumber, std::unique_ptr<cv::Mat> img, const std::string& imgName) {
	ArucoDetection arucoDetector;
	Markers markers;
	arucoDetector.detect(*img, markers);
	for (auto& [id, marker] : markers) {
		marker.pose() = marker.T().inv();
	}
	Frame frame(frameNumber, imgName, std::move(img), markers);
	bool stop = false;
	if (_source!=Source::LIVE_CAPTURE|| _isSnap) 
		if (_isBA) BA(frame);
	if (!_isParallel) {
		_window.updateWindow(frame);
		frame.show();
		stop = getCommand(frame);
		_window.show();
		if (_window.isFreezeViz = _isMonitor || _window.isFreezeViz) freezing(frame);
	}
	recorder.addFrame(frame, _isSnap, _isParallel );
	if(frame.img) frame.img.release();
	return stop;
}

void Analysis::freezing(Frame& frame) {
	while (_window.isFreezeViz) {
		_window.show();
		getCommand(frame);
	}
}

void Analysis::BA(Frame& frame){
	int vertexID = 1;
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> > Block;
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
	Block* solver_ptr = new Block(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(solver);

	g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
	vSE3->setEstimate(g2o::SE3Quat());
	vSE3->setFixed(true);
	vSE3->setId(0);
	optimizer.addVertex(vSE3);

	for (const auto& [id, marker] : frame.markers) {
		g2o::VertexSE3Expmap* vSE3_marker = new g2o::VertexSE3Expmap();
		vSE3_marker->setEstimate(g2o::SE3Quat(affineToSE3(marker.pose())));
		vSE3_marker->setId(vertexID);
		vSE3_marker->setFixed(false);
		optimizer.addVertex(vSE3_marker);
		for (int i = 0; i < 4; i++) {
			g2o::EdgeSE3ProjectMarker* edge = new g2o::EdgeSE3ProjectMarker();
			edge->fx = Params::kmat.at<double>(0, 0);
			edge->fy = Params::kmat.at<double>(1, 1);
			edge->cx = Params::kmat.at<double>(0, 2);
			edge->cy = Params::kmat.at<double>(1, 2);
			edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vSE3));
			edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(vertexID)));
			edge->setMeasurement(Eigen::Vector2d(marker.corners.at(i).x, marker.corners.at(i).y));
			edge->SetMarkerPoint(cvPoint2EigenVec<cv::Point3d, Eigen::Vector3d>(marker.markerCoordCorners().at(i)));
			edge->setInformation(Eigen::Matrix2d::Identity());
			optimizer.addEdge(edge);
		}
		vertexID++;
	}


	optimizer.setVerbose(Settings::dispBA);
	optimizer.initializeOptimization();
	optimizer.optimize(1500);

	vertexID = 1;
	for (auto& [id, marker] : frame.markers) {
		if (Settings::dispBA)std::cout << "ID" << id << " Before:" << std::endl << marker.pose().matrix << std::endl;
		std::cout << marker.err() << std::endl;
		std::cout << marker.reprojectionError() << std::endl;
		g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(vertexID++));
		marker.pose() = SE3ToAffine(pose->estimate());
		if (Settings::dispBA)std::cout << "ID" << id << " After:" << std::endl << marker.pose().matrix << std::endl;
		std::cout << marker.err() << std::endl;
	}
	std::cout << std::endl;
	

}

void Analysis::outputResults(const std::string& path) const{
	recorder.output(path);
}

void Analysis::rescue(Frame& frame){
	ArucoDetectionSetting arucoDetectionSetter;
	arucoDetectionSetter.tune(frame.frameNumber, std::make_unique<cv::Mat>(*frame.img), frame.imgName);
	for (auto& [id, marker] : arucoDetectionSetter.markers()) {
		marker.pose() = marker.T().inv();
		auto it = frame.markers.find(id);
		if (it == frame.markers.end()) frame.markers.emplace(id, marker);
		else {
			frame.markers.erase(it);
			frame.markers.emplace(id, marker);
		}
	}
	if (_isBA) BA(frame);
	frame.show();
	_window.updateWindow(frame);
}



void Analysis::adjustSetting(Frame& frame){
	char option = -1;
	std::cout << "set what?  [1] Pose Estimation [2] ArUco Detection" << std::endl;
	std::cin >> option;
	try {
		switch (option) {
		case '1': {
			char option2 = -1;
			std::string value = "";
			std::cout << "set what?  [1]unambiguousErrorRaioThresh,[2] projectionErrorThresh" << std::endl;
			std::cin >> option2;
			switch (option2) {
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
				std::cout << "wrong input";
			}
			break;
		}
		case '2': {
			rescue(frame);
		}
		default:
			std::cout << ("wrong input");
			return;
		}
		std::cout << "change succesfully" << std::endl;
	}
	catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		std::cout << "change aborted" << std::endl;
	}
}

bool Analysis::getCommand(Frame& frame) {
	if (_window.isTerminate||_window().wasStopped()) {
		return true;
	}
	if (_window.isAdjustSetting) {
		adjustSetting(frame);
		_window.isAdjustSetting = false;
	}
	if (_window.isSnap) {
		_isSnap = true;
		_window.isSnap = false;
	}
	if (_window.isRescue) {
		int oldAdaptiveThreshBlockSize = Settings::adaptiveThreshBlockSize;
		int adaptiveThreshC = Settings::adaptiveThreshC;
		int approxPolyDPEpsilonRatio = Settings::approxPolyDPEpsilonRatio;
		int cornerRefineWinsizeRatio = Settings::cornerRefineWinsizeRatio;
		int minArea = Settings::minArea;
		Polys detectionRegions = Settings::detectRegions;
		rescue(frame);
		Settings::adaptiveThreshBlockSize = oldAdaptiveThreshBlockSize;
		Settings::adaptiveThreshC = adaptiveThreshC;
		Settings::approxPolyDPEpsilonRatio = approxPolyDPEpsilonRatio;
		Settings::cornerRefineWinsizeRatio = cornerRefineWinsizeRatio;
		Settings::minArea = minArea;
		Settings::detectRegions = detectionRegions;
		_window.isFreezeViz = true;
		_window.isRescue = false;
	}
	return false;
}




