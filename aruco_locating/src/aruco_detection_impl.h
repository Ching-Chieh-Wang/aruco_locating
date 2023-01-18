#ifndef ARUCO_DETECTION_IMPL_H
#define ARUCO_DETECTION_IMPL_H

//ArucoDetection ���H���{��
class ArucoDetectionImpl {
protected:
	//�ˬd�Կ�H�̥~��O�_�O�¦�
	static bool boarderIsBlack(cv::Mat& binaryCode, int fullsize);
	//��4�Ө��I���ɰw�ƦC
	static void makePointsOrder(std::vector<cv::Point2i>& points);
	//�ˬd�O�_�Կ�������~����
	static bool contourIsOuter(const std::vector<cv::Vec4i>& hierachy, const int idx);
	//�إ߼v���B�n(�����d��)
	void createMask(const cv::Mat&img,const Polys& detectRegions, cv::Mat& masked);

	MarkerId idMatcher(const cv::Mat& warped, PolyF& candidatef);

};
#endif