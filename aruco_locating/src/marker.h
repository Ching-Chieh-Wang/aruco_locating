#ifndef MARKER_H
#define MARKER_H
class Marker :public PnPObj{
private:
	cv::Affine3d _pose;
	//查看前幾個frame中此marker的rvec,tvec
	bool getPoseHistory(cv::Mat& guessRvec, cv::Mat& guessTvec) const;
	//得到此marker與frame的相對位姿
	void solvePnP();
	static std::unordered_map<MarkerId, std::pair<cv::Mat, cv::Mat>> markerTHistories;
public:
	inline cv::Affine3d& pose() {
		return _pose;
	}
	inline const cv::Affine3d& pose() const{
		return _pose;
	}
	const MarkerId id = -1;
	const double size=-1;
	//Marker在圖像中的四個角點
	const std::vector<cv::Point2f> corners;
	//於map中顯示marker資訊
	void show(cv::Mat& img)const;
	inline  std::vector<cv::Point3d> markerCoordCorners() const
	{
		assert(size > 0);
		return { cv::Point3d(-size / 2,size / 2,0), cv::Point3d(size / 2,size / 2,0),cv::Point3d(size / 2,-size / 2,0),cv::Point3d(-size / 2,-size / 2,0) };
	}
	cv::Point3d worldCoordCenter()const;
	Clouds worldCoordCorners()const;
	Marker(const MarkerId id, const double& size,  const std::vector<cv::Point2f>& corners);

};

std::unordered_map<MarkerId, std::pair<cv::Mat,cv::Mat>> Marker::markerTHistories = std::unordered_map<MarkerId, std::pair<cv::Mat, cv::Mat>>{};





#endif