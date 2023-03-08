#ifndef DICTIONARY_H
#define DICTIONARY_H

#include "../pch.h"
#include "types.h"


class Dictionary {
public:
	///ArUco字典種賴
	enum class DictType {
		ERROR =-1,
		ARUCO = 0,
		ARUCO_MIP_36h12 = 1,
	};
	//紋路比對字典
	int find(const std::bitset<64>& binary)const;
	//字典種類
	DictType type = DictType::ERROR;
	Dictionary(const DictType type,const double& markerSize, const std::map<MarkerId,double>& speiclaMarkerSizes);
	//查詢標記的尺寸
	double markerSize(const MarkerId id)const;
	///Aruco是bitSize*bitSize格子組成
	int bitSize;
private:
	///所有Codes
	std::unordered_map<std::bitset<64>, MarkerId> bitCodes;
	//普遍標記邊長
	double _markerSize;
	//特例標記邊長
	std::unordered_map<MarkerId, double> _specialMarkerSizes;
};
#endif 



