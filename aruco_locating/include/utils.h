#ifndef UTILS_H
#define UTILS_H

#include "pch.h"


cv::Scalar randColor();
cv::Affine3d rvecTvec2T(const cv::Mat& rvec, const cv::Mat& tvec);
std::tuple<cv::Mat, cv::Mat> T2RvecTvec(const cv::Affine3d& T);
Eigen::Matrix4d cvAffine2EigenMat(const cv::Affine3d& pose);
cv::Affine3d SE3ToAffine(const g2o::SE3Quat& SE3);
g2o::SE3Quat affineToSE3(const cv::Affine3d& affine);
cv::Point3d getIntersect(const cv::Vec3d & planeVec, const cv::Point3d & objPoint, const cv::Point3d & objPointPoint2, const cv::Point3d & planePoint);

template< class Eigen, class CV>
 CV eigenVec2CvPoint(const Eigen& point) {
	return CV(point.x(), point.y(), point.z());
}
template< class CV, class Eigen>
Eigen cvPoint2EigenVec(const CV& point) {
	return Eigen(point.x, point.y, point.z);
}



Eigen::Vector3d getCornerCenter(const std::vector<Eigen::Vector3d>& corners);


cv::Vec2f rotationMatrixToEulerAngles(const cv::Mat& R);


#endif // !