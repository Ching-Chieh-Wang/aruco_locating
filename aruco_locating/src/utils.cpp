#include "pch.h"
#include "utils.h"

cv::Scalar randColor() {
	cv::Scalar color(0,0,0);
	int n = 200;
	for (int i = 0; i < 3; i++) {
		if ((n - 1) == RAND_MAX) {
			return rand();
		}
		else {
			/* 計算可以被整除的長度 */
			long end = RAND_MAX / n;
			assert(end > 0L);
			end *= n;

			/* 將尾端會造成偏差的幾個亂數去除，
			   若產生的亂數超過 limit，則將其捨去 */
			int r;
			while ((r = rand()) >= end);
			color[i] = r % n;
		}
	}
	return color;
}
cv::Affine3d rvecTvec2T(const cv::Mat& rvec, const cv::Mat& tvec) {
	cv::Mat R;
	cv::Rodrigues(rvec, R);
	return cv::Affine3d(R, tvec);
}

std::tuple<cv::Mat, cv::Mat> T2RvecTvec(const cv::Affine3d& T) {
	cv::Mat rvec;
	cv::Rodrigues(T.rotation(), rvec);
	return std::tuple<cv::Mat, cv::Mat>(rvec, T.translation());
}






Eigen::Matrix4d cvAffine2EigenMat(const cv::Affine3d& pose) {
	Eigen::Matrix4d pose_;
	cv::cv2eigen(cv::Mat(pose.matrix), pose_);
	return pose_;
}

cv::Affine3d SE3ToAffine(const g2o::SE3Quat &SE3) {
	cv::Matx33d R;
	cv::Vec3d tvec;
	cv::eigen2cv(Eigen::Matrix3d(SE3.rotation()), R);
	cv::eigen2cv(SE3.translation(), tvec);
	return cv::Affine3d(R, tvec);
}

g2o::SE3Quat affineToSE3(const cv::Affine3d &affine) {
	Eigen::Matrix3d R;
	Eigen::Vector3d tvec;
	cv::cv2eigen(affine.rotation(), R);
	cv::cv2eigen(affine.translation(), tvec);
	return g2o::SE3Quat(R, tvec);
}




cv::Point3d getIntersect(const cv::Vec3d& planeVec, const cv::Point3d& objPoint, const cv::Point3d& objPointPoint2, const cv::Point3d& planePoint) {
	double t = planeVec.dot(planePoint - objPoint) / planeVec.dot(objPointPoint2 - objPoint);
	return objPoint + t * (objPointPoint2 - objPoint);
}




Eigen::Vector3d getCornerCenter(const std::vector<Eigen::Vector3d> &corners) {
	assert(corners.size() == 4);
	Eigen::Vector3d center(0,0,0);
	for (const Eigen::Vector3d &corner:corners) {
		center.x() += corner.x();
		center.y() += corner.y();
		center.z() += corner.z();
	}
	center.x() = center.x() / 4;
	center.y() = center.y() / 4;
	center.z() = center.z() / 4;
	return center;
}



cv::Vec2f rotationMatrixToEulerAngles(const cv::Mat& R) {
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	return cv::Vec2f(180 - abs(atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * 57.3), abs(atan2(-R.at<double>(2, 0), sy) * 57.3));
}


