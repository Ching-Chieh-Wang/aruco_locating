#ifndef PNP_OBJ_H
#define PNP_OBJ_H

#include "pch.h"
#include "settings.h"

class PnPObj {


protected:
	//pnp兩個結果的誤差
	std::vector<float> _reprojectionErrors;
	//PNP 得到的兩組rvec解
	std::vector<cv::Mat> _rvecs;
	//PNP 得到的兩組tvec解
	std::vector<cv::Mat> _tvecs;

public:
	friend class PoseEstimation;
	//marker座標轉相機座標
	const cv::Affine3d T() const;
	const cv::Affine3d T(const int idx) const;

	virtual inline const float reprojectionErrorRatio()const
	{
		return _reprojectionErrors[biggerErrorIdx()] / reprojectionError();
	}
	inline const float reprojectionError() const
	{
		return _reprojectionErrors[smallerErrorIdx()];
	}
	inline const float reprojectionError(int idx) const
	{
		return _reprojectionErrors[idx];
	}

	inline int const smallerErrorIdx() const
	{
		return _reprojectionErrors[0] > _reprojectionErrors[1] ? 1 : 0;
	}
	inline int const biggerErrorIdx()const
	{
		return smallerErrorIdx() == 0 ? 1 : 0;
	}
	inline bool unambiguous() const {
		return  reprojectionErrorRatio() > Settings::unambiguousErrorRaioThresh;
	}
	inline bool bigError() const {
		return _reprojectionErrors[smallerErrorIdx()] > Settings::projectionErrorThresh;
	}
	//pnp結果是否可信
	inline bool accessible() const
	{
		return unambiguous() && !bigError();
	}
	virtual inline const cv::Mat& rvec() const
	{
		if (enforceIdx!=-1)  return _rvecs[enforceIdx];
		else return _rvecs[smallerErrorIdx()];
	}
	virtual inline const cv::Mat& rvec(const int idx) const
	{
		return _rvecs[idx];
	}
	const cv::Mat rotation()const;
	const cv::Mat rotation(const int idx)const;

	const cv::Mat rotation(bool minError) const;

	virtual inline const cv::Mat& tvec() const
	{
		if (enforceIdx != -1)  return _tvecs[enforceIdx];
		else return _tvecs[smallerErrorIdx()];
	}
	virtual inline const cv::Mat& tvec(const int idx) const
	{
		return _tvecs[idx];
	}
	int enforceIdx=-1;

};
#endif