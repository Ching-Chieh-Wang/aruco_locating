#ifndef ANALYSIS_H
#define ANALYSIS_H
class Analysis {
public:
	//�i����R
	void run(const Source source, const std::string& sourcePath,const bool isParallel,const bool isMonitor,const bool isBA);
	//��X���R���G
	void outputResults(const std::string& path) const;
private:
	//�z�L�]�w���쥻�S�������쪺aruco�аO�i�H�Q������
	void rescue(Frame& frame);
	Source _source;
	//�T����ı����
	VIZWindow _window;
	//�վ�]�w
	void adjustSetting(Frame& frame);
	//�ˬd�ϥΪ̦��U���O�ð��X�۹����ʧ@
	bool getCommand(Frame& frame);
	Record recorder;
	//�x�s���W�@�V
	Frame lastFrame;
	//���R�@�i�Ӥ�
	bool analysis(const FrameNumber frameNumber, std::unique_ptr<cv::Mat> img, const std::string& imgName);
	//�Ȱ����R
	void freezing(Frame& frame);
	//�O�_���n�D����B��
	bool _isParallel = false;
	//�O�_���n�D�ʷ��аO����������
	bool _isMonitor = false;
	//�O�_���n�D�}��BA�u��
	bool _isBA = false;
	//�O�_�n�D���
	bool _isSnap = false;
	//BA�̨Τ�
	void BA(Frame& frame);

};

#endif // !ANALYSIS_H

