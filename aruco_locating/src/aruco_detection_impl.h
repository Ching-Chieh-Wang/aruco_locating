#ifndef ARUCO_DETECTION_IMPL_H
#define ARUCO_DETECTION_IMPL_H

//ArucoDetection ���H���{��
class ArucoDetectionImpl {
protected:
	//�Ǧ��
	cv::Mat gray;
	//��t��
	cv::Mat thresholded;
	//�ˬd�аO�Կ�H�Ӥ��̥~��O�_�O�¦�
	bool boarderIsBlack(cv::Mat& binaryCode, int fullsize);
	// �إ߰����d��B�n���Ϩϰ����аO�t�ץ[��
	void createMask();
	//�z��аO�Կ�H�����O�_������|��ΥB���Y�]�B�j�p�A��
	bool polyfilter(Poly& candidate, std::vector<cv::Vec4i>& hierarchy, int hierarchyId);
	///�W�i�аO��Ӥ������I�w����
	void cornerRefine(Poly& candidate, PolyF& candidatef);
	//��4�Ө��I���ɰw�ƦC
	void makePointsOrder(std::vector<cv::Point2i>& points);
	//�ˬd�O�_�Կ�������~����
	bool contourIsOuter(const std::vector<cv::Vec4i>& hierachy, const int idx);
	//�إ߼v���B�n(�����d��)
	void createMask(const cv::Mat&img,const Polys& detectRegions, cv::Mat& masked);
	//���аO�Ӥ���ID�A�Y�P�_�ӷӤ��D�аO�Ӥ��^��-1
	MarkerId idMatcher(const cv::Mat& warped, PolyF& candidatef);
	//�N�ϰ�i�楿��v��g
	void warp(PolyF& candidate, cv::Mat& warped);
	void contrastAdjust(const cv::Mat& img, cv::Mat& adjustedImg, const int& contrastFactor);
	int contrastPixelWiseAdjust(const int &val, const int &contrastFactor);

};
#endif