#include "pch.h"
#include "pnp_obj.h"






const cv::Mat PnPObj::rotation() const{
	cv::Mat R;
	cv::Rodrigues(rvec(), R);
	return R;
}

const cv::Mat PnPObj::rotation(const int idx) const{
	cv::Mat R;
	cv::Rodrigues(rvec(idx), R);
	return R;
}

const cv::Affine3d PnPObj::T()const {
	return cv::Affine3d(rotation(), tvec());
}

const cv::Affine3d PnPObj::T(const int idx) const
{
	return cv::Affine3d(rotation(idx), tvec(idx));
}

