#include "aruco_detection_impl.h"


bool ArucoDetectionImpl::boarderIsBlack(cv::Mat& binaryCode, int fullsize) {
	for (int y = 0; y < fullsize; y++)
	{
		int inc = fullsize - 1;
		if (y == 0 || y == fullsize - 1)
			inc = 1;  // for first and last row, check the whole border
		for (int x = 0; x < fullsize; x += inc)
			if (binaryCode.at<uchar>(y, x) != 0)  return false;
	}
	return true;
}

void ArucoDetectionImpl::makePointsOrder(std::vector<cv::Point2i>& points) {
	int dx1 = points.at(1).x - points.at(0).x;
	int dy1 = points.at(1).y - points.at(0).y;
	int dx2 = points.at(2).x - points.at(0).x;
	int dy2 = points.at(2).y - points.at(0).y;
	int o = (dx1 * dy2) - (dy1 * dx2);
	if (o < 0) {
		std:swap(points.at(1), points.at(3));
	}
	return;
}

bool ArucoDetectionImpl::contourIsOuter(const std::vector<cv::Vec4i>& hierachy, const int idx){
	int checkCount = 0;
	int currentIdx = idx;
	while (hierachy[currentIdx][3] != -1) {
		currentIdx = hierachy[currentIdx][3];
		checkCount++;
	}
	return checkCount % 2 == 0 ? true : false;
}

void ArucoDetectionImpl::createMask(const cv::Mat& img, const Polys& regions, cv::Mat& masked) {
	cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
	cv::fillPoly(mask, regions, cv::Scalar(255, 255, 255));
	bitwise_and(img, img, masked, mask);
}

MarkerId  ArucoDetectionImpl::idMatcher(const cv::Mat& warped, PolyF& candidatef) {
	//�������̷�DictionaryBitsize�M�wwholeBlockSize�A+2�O�]���̥~����䦳�@�w�]���¦�ϰ�
	cv::Mat otsued;
	cv::threshold(warped, otsued, 0, 255, cv::THRESH_OTSU);
	int fullsize = Params::dictionary.bitSize + 2;
	//�h�j�M�b�C��Block�����h�֥զ�px�A�H�ΨC��block�����h��px
	cv::Mat whitePx(fullsize, fullsize, CV_32SC1);
	cv::Mat allPx(fullsize, fullsize, CV_32SC1);
	whitePx.setTo(cv::Scalar::all(0));
	allPx.setTo(cv::Scalar::all(0));
	for (int y = 0; y < otsued.rows; y++) {
		const uchar* ptr = otsued.ptr<uchar>(y);
		int my = float(fullsize) * float(y) / float(otsued.rows);
		for (int x = 0; x < otsued.cols; x++) {
			int mx = float(fullsize) * float(x) / float(otsued.cols);
			if (ptr[x] != 0) whitePx.at<int>(my, mx)++;
			allPx.at<int>(my, mx)++;
		}
	}
	cv::Mat binaryMat = cv::Mat(fullsize, fullsize, CV_8UC1);
	//�Y�C��block���զ�px�ӼƤj����block��px�`�Ƥ��@�b�A�NbinaryCode����block���ȳ]��1�A�Ϥ���0
	for (int y = 0; y < fullsize; y++)
		for (int x = 0; x < fullsize; x++) {
			if (whitePx.at<int>(y, x) > allPx.at<int>(y, x) / 2) binaryMat.at<uchar>(y, x) = 1;
			else binaryMat.at<uchar>(y, x) = 0;
		}
	//�p�G�o�{�̥~�򤣬O�¦�N���L�o��Candidates
	if (!boarderIsBlack(binaryMat, fullsize)) return -1;
	//�}�l�N�Ӥ��Pdictionary�i����A�p�G�@��match ����dictionary ��bitCodes�A�N����arucobinary�A�������4���C
	int rotation = 0;
	int id = -1;
	while (id == -1 && rotation < 4) {
		std::bitset<64> arucoBinary;
		int cursor = 0;
		//�����̥~��¦�ϰ�A�]�N�Obinary code ����ئa�a�AbinaryCodeMat �s��Mat�����AbinaryCode �s���G�i��Ʀr
		for (int x = Params::dictionary.bitSize; x > 0; x--)
			for (int y = Params::dictionary.bitSize; y > 0; y--)
				arucoBinary[cursor++] = binaryMat.at<uchar>(x, y);
		id = Params::dictionary.find(arucoBinary);
		if (id != -1) {
			for (int i = 0; i < rotation; i++) {
				candidatef.insert(candidatef.begin(), candidatef.back());
				candidatef.pop_back();
			}
			return id;
		}
		else {
			cv::Mat rotatedBinaryMat(fullsize, fullsize, CV_8UC1);
			cv::rotate(binaryMat, binaryMat, cv::ROTATE_90_CLOCKWISE);
			rotation++;
		}
	}
	return -1;
}

