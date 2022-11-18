
#include "settings.h"

void Settings::load(const std::string&path) {
	_path = path;
	//�{��jsonŪ����
	Json::Value loader = Json::Value(parse(path));

	///
	/// 	ArUco detection
	/// 

	

	///�����a����t��marker�A��t���Z��
	cutBoarder = loader["arucoDetection"]["cutBoarder"].asInt();
	if (cutBoarder < 0) throw("[Settings.json] cutBorder cannot be <0");
	///�֭�-C�A�Ʀr�V�p�V����ѨM���v���D�A���t�׶V�C
	adaptiveThreshC = loader["arucoDetection"]["adaptiveThreshC"].asInt();
	if (adaptiveThreshC < 0) throw("[Settings.json] adaptiveThreshC cannot be <0");
	///filter size�erow���X��
	adaptiveThreshBlockSize = loader["arucoDetection"]["adaptiveThreshBlockSize"].asInt();
	if (adaptiveThreshBlockSize < 0) throw("[Settings.json] adaptiveThreshBlockSize cannot be <0");
	if(adaptiveThreshBlockSize%2==0) throw("[Settings.json] adaptiveThreshBlockSize("+std::to_string(adaptiveThreshBlockSize)+") must be odd");
	//markerCandiates���n�ݶW�L��i�v���d�򤧤�v
	minArea = loader["arucoDetection"]["minArea"].asInt();
	if (minArea < 0) throw("[Settings.json] minArea cannot be <0");
	//ApproxPolyDP �~�t�����d��epsilon�Z��:contour���������v
	approxPolyDPEpsilonRatio = loader["arucoDetection"]["approxPolyDPEpsilonRatio"].asInt();
	if (approxPolyDPEpsilonRatio < 0) throw("[Settings.json] approxPolyDPEpsilonRatio cannot be <0");
	//CornerRefine ���j��������aruco�@���������v
	cornerRefineWinsizeRatio = loader["arucoDetection"]["cornerRefineWinsizeRatio"].asInt();
	if (cornerRefineWinsizeRatio < 0) throw("[Settings.json] cornerRefineWinsizeRatio cannot be <0");
	//�������d��
	detectRegions = getDetectRetgions(loader);

	///
	/// Pose estimation
	/// 

	//unambiguous���֭�(eq14)�A�j��error�b���l
	unambiguousErrorRaioThresh = loader["poseEstimation"]["unambiguousErrorRaioThresh"].asFloat();
	if (unambiguousErrorRaioThresh < 0) throw("[Settings.json] unambiguousErrorRaioThresh cannot be <0");
	// pnp ���~�t�֭�
	projectionErrorThresh = loader["poseEstimation"]["projectionErrorThresh"].asFloat();
	if (projectionErrorThresh < 0) throw("[Settings.json] projectionErrorThresh cannot be <0");

	///display
	dispBA = loader["dispBA"].asBool();
	std::cout << "Load settings succesfully" << std::endl;
}

void Settings::save(const std::string& path){
	if (path != "") _path=path;
	Json::Value saver = Json::Value();
	saver["arucoDetection"]["cutBoarder"] = cutBoarder;
	saver["arucoDetection"]["adaptiveThreshBlockSize"] = adaptiveThreshBlockSize;
	saver["arucoDetection"]["adaptiveThreshC"] = adaptiveThreshC;
	saver["arucoDetection"]["minArea"] = minArea;
	saver["arucoDetection"]["approxPolyDPEpsilonRatio"] = approxPolyDPEpsilonRatio;
	saver["arucoDetection"]["cornerRefineWinsizeRatio"] = cornerRefineWinsizeRatio;
	saver["arucoDetection"]["detectRegions"] = saveDetectionRegions();
	saver["poseEstimation"]["unambiguousErrorRaioThresh"] = unambiguousErrorRaioThresh;
	saver["poseEstimation"]["projectionErrorThresh"] = projectionErrorThresh;
	saver["dispBA"] = dispBA;
	Json::StyledWriter sw;
	std::ofstream jsonFile(_path, std::ios::out);
	if (!jsonFile.is_open()) {
		throw("Error saving settings to json file");
	}
	jsonFile << sw.write(saver);
	jsonFile.close();
	std::cout << "Save settings succesfully" << std::endl;
}

Polys Settings::getDetectRetgions(const Json::Value & loader){
	Polys detectRegions;
	if (loader["arucoDetection"]["detectRegions"].isNull()) {
		detectRegions = Polys{ Poly{ cv::Point2i(0, 0), cv::Point2i(0, Params::imgSize.height-1), cv::Point2i(Params::imgSize.width-1, Params::imgSize.height-1), cv::Point2i(Params::imgSize.width - 1, 0) } };
	}
	else {
		for (const auto& polyJ : loader["arucoDetection"]["detectRegions"]) {
			Poly poly;
			for (const auto& point : polyJ) {
				if (point[0].asInt() < 0 || point[1].asInt() < 0) throw "[Settings.json] Detection region cannot <0]";
				if (point[0].asInt() > Params::imgSize.width) throw "[Settings.json] Detection region width(" + point[0].asString() + ")cannot exceed frame size(" + std::to_string(Params::imgSize.width) + ")]";
				if (point[1].asInt() > Params::imgSize.height) throw "[Settings.json] Detection region height(" + point[1].asString() + ")cannot exceed frame size(" + std::to_string(Params::imgSize.height) + ")]";
				poly.emplace_back(cv::Point2i(point[0].asInt(), point[1].asInt()));
			}
			detectRegions.emplace_back(poly);
		}
	}
	return detectRegions;
}

Json::Value Settings::saveDetectionRegions(){
	Json::Value polys;
	for (const Poly& detectionRegion : detectRegions) {
		Json::Value poly;
		for (const cv::Point2i& point : detectionRegion) {
			Json::Value pointJ;
			pointJ.append(point.y);
			pointJ.append(point.x);
			poly.append(pointJ);
		}
		polys.append(poly);
	}
	return polys;
}
















