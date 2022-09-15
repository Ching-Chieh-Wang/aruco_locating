#ifndef PARAMS_H
#define PARAMS_H
//�`�νվ�Ѽ�
class Params {
public:
	//�۾����ѯx�}
	static cv::Mat kmat;
	//�۾�����x�}
	static cv::Mat distmat;
	static Dictionary dictionary;
	//�۾��g�L����ץ��O�d����px�����ѯx�}
	static cv::Mat undistKmat;
	static cv::Mat map1, map2;
	static cv::Size imgSize;
	static double markerSize;
	//�w�����쫺��marker
	static std::unordered_map<int, double> markers;
	static void load(const std::string& path);
private:
	static std::map<MarkerId, double> getSpecialMarkerSizes(const Json::Value& loader);
};


cv::Mat Params::kmat = cv::Mat();
cv::Mat Params::distmat = cv::Mat();
cv::Size Params::imgSize = cv::Size();
cv::Mat Params::undistKmat = cv::Mat();
cv::Mat Params::map1 = cv::Mat();
cv::Mat Params::map2 = cv::Mat();
double Params::markerSize = -1;
Dictionary Params::dictionary(Dictionary::DictType::ERROR,-1,std::map<MarkerId,double>{});
std::unordered_map<int, double> markers{};
#endif