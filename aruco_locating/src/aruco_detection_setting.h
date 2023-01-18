#ifndef ARUCO_DETECTION_SETTING_H
#define ARUCO_DETECTION_SETTING_H

class ArucoDetectionSetting :private ArucoDetectionImpl {
public:
	void run(const Source source, const std::string& sourcePath);
	bool tune(const FrameNumber frameNumber, std::unique_ptr<cv::Mat> img, const std::string& imgName);
	Markers markers();
private:
	//�Ӥ����
	cv::Mat img;
	//�����쪺����
	cv::Mat contoursImg;
	//���I�w���u�Ƶ��G
	cv::Mat cornerRefineImg;
	//��쪺�Ҧ�����
	Polys contours;
	//�æ��аO��
	Polys candidates;
	//��������
	std::vector<cv::Vec4i> contourHierarchy;
	//�վ�Ϥ�������t���Ѽ�
	static void setThreshParams(int, void*);
	//�վ�h��Ϊ��ʽ�z��
	static void setPolyParams(int ,void*);
	//�վ㨤�I�w���u�ƪ��j�������j�p
	static void setCornerRefineParams(int, void*);
	//�x�s�����аO���Ѽ�
	void save();
	Markers _markers;
};


#endif 




