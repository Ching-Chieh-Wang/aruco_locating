#ifndef PNP_OBJ_H
#define PNP_OBJ_H

#include "pch.h"
#include "settings.h"

class PnPObj {


protected:
	//pnp��ӵ��G���~�t
	std::vector<float> _reprojectionErrors;
	//PNP �o�쪺���rvec��
	std::vector<cv::Mat> _rvecs;
	//PNP �o�쪺���tvec��
	std::vector<cv::Mat> _tvecs;

public:
	friend class PoseEstimation;
	//marker�y����۾��y��
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
	//pnp���G�O�_�i�H
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