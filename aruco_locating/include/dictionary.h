#ifndef DICTIONARY_H
#define DICTIONARY_H

#include "../pch.h"
#include "types.h"


class Dictionary {
public:
	///ArUco�r��ؿ�
	enum class DictType {
		ERROR =-1,
		ARUCO = 0,
		ARUCO_MIP_36h12 = 1,
	};
	//�������r��
	int find(const std::bitset<64>& binary)const;
	//�r�����
	DictType type = DictType::ERROR;
	Dictionary(const DictType type,const double& markerSize, const std::map<MarkerId,double>& speiclaMarkerSizes);
	//�d�߼аO���ؤo
	double markerSize(const MarkerId id)const;
	///Aruco�ObitSize*bitSize��l�զ�
	int bitSize;
private:
	///�Ҧ�Codes
	std::unordered_map<std::bitset<64>, MarkerId> bitCodes;
	//���M�аO���
	double _markerSize;
	//�S�ҼаO���
	std::unordered_map<MarkerId, double> _specialMarkerSizes;
};
#endif 



