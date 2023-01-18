#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

class ArucoDetection :private ArucoDetectionImpl {
public:
	void detect(const cv::Mat & image, Markers& markers);
private:
	//��Ϧ�
	cv::Mat gray;
	//adaptive threshold �᪺��
	cv::Mat thresholded;
	//findContour �o�쪺�Ҧ�contour����
	cv::Mat imageContours;
	//approxpolyDP �o�쪺�Ҧ��|���contour����
	cv::Mat polyFilteredImg;
	//�Ҧ������쪺marker
	Markers markers;
	// �إ߰����d��B�n
	void createMask();
	//�z��contour������|��ΥB���Y�]�B�j�p�W�L�@�w�ȥB�L�o�j�p�Ӫ��
	bool polyfilter(Poly& candidate, std::vector<cv::Vec4i>& hierarchy, int i);
	///�]���qapproxpolyDP�o�쪺���I���̰���ץu�������A�M��cornerSubPix�o��ȹ������I
	void cornerRefine(Poly& candidate, PolyF& candidatef);
	//�N�æ���marker�ϰ�i�楿��v��g
	void warp(PolyF& candidate, cv::Mat& warped);
};
#endif 



