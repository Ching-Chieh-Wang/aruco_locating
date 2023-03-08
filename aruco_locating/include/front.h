#ifndef FRONT_H
#define FRONT_H
#include "types.h"
#include "analysis.h"

// �e�ݤ���
class Front {
public:
	Front(const std::string& paramsPath="params.json", const std::string& settingsPath="settings.json");
	//�վ㰻��aruco detection���Ѽ�
	void arucoDetectionSet(const Source source, const std::string& sourcePath);
	// �}�l�]
	void run(const Source source, const std::string& sourcePath, const bool BA, const bool monitor, const bool parallel=false);
	void save(const std::string path="")const;
	inline void help() const{
		std::cout << "�����z�LsetSource()��ܼv���ӷ��P����|\n�ϥ�narucoDetectionSet�ӽվ㰻��aruco���Ѽ�\n�S���D��ϥ�run()�}�l���R�A�Y�n�������R�����T����ı�����Φb�����W���Uexc��\n�ϥ�save()�x�s���R�����G" << std::endl;
	}
private:
	Analysis analyzer;


};
#endif