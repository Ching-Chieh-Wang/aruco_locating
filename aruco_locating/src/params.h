#ifndef PARAMS_H
#define PARAMS_H
//�`�νվ�Ѽ�
class Params {
public:
	static std::string filePath;
	//�۾����ѯx�}
	static cv::Mat kmat;
	//�۾�����x�}
	static cv::Mat distmat;
	static Dictionary dictionary;
	static cv::Size imgSize;
	static double markerSize;
	//�w�����쫺��marker
	static std::unordered_map<int, double> markers;
	static void load(const std::string& path);
private:
	static std::map<MarkerId, double> getSpecialMarkerSizes(const Json::Value& loader);
};

std::string Params::filePath = "";
cv::Mat Params::kmat = cv::Mat();
cv::Mat Params::distmat = cv::Mat();
cv::Size Params::imgSize = cv::Size();
double Params::markerSize = -1;
Dictionary Params::dictionary(Dictionary::DictType::ERROR,-1,std::map<MarkerId,double>{});
std::unordered_map<int, double> markers{};
#endif