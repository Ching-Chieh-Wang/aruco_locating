
#ifndef SETTINGS_H
#define SETTINGS_H

#include "pch.h"
#include "json_reader.h"
#include "types.h"


class Settings :private JsonReader{
public:

	//對比度調整係數
	static int contrastFactor;
	///忽略靠近邊緣多少px的marker
	static int cutBoarder;
	///窗格大小，越大越容易將marker附近的類似黑色區域也考慮為marker邊界，越小的話marker太小會偵測不到
	static int adaptiveThreshBlockSize;
	///閥值-C，數字越小越易於解決陰影問題，但速度越慢
	static int adaptiveThreshC;
	//markerCandiates佔整張影像之比率閥值
	static int minArea;
	//ApproxPolyDP 誤差接受範圍epsilon距離:contour的邊長的比率
	static int approxPolyDPEpsilonRatio;
	//CornerRefine 的搜索視窗佔aruco一格邊長的比率
	static int cornerRefineWinsizeRatio;
	static Polys detectRegions;

	static float unambiguousErrorRaioThresh;
	static float projectionErrorThresh;
	static bool dispBA;
	static void load(const std::string& path);
	static void save(const std::string& path="");
private:
	static std::string _path;
	static Polys getDetectRetgions(const Json::Value& loader);
	static Json::Value saveDetectionRegions();

};


#endif 


