
#ifndef SETTINGS_H
#define SETTINGS_H

#include "pch.h"
#include "json_reader.h"
#include "types.h"


class Settings :private JsonReader{
public:

	//���׽վ�Y��
	static int contrastFactor;
	///�����a����t�h��px��marker
	static int cutBoarder;
	///����j�p�A�V�j�V�e���Nmarker���������¦�ϰ�]�Ҽ{��marker��ɡA�V�p����marker�Ӥp�|��������
	static int adaptiveThreshBlockSize;
	///�֭�-C�A�Ʀr�V�p�V����ѨM���v���D�A���t�׶V�C
	static int adaptiveThreshC;
	//markerCandiates����i�v������v�֭�
	static int minArea;
	//ApproxPolyDP �~�t�����d��epsilon�Z��:contour���������v
	static int approxPolyDPEpsilonRatio;
	//CornerRefine ���j��������aruco�@���������v
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


