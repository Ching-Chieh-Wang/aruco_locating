

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


//json檔案擷取器
Json::Value parse(const std::string jsonPath) {
	std::ifstream ifs;
	ifs.open(jsonPath);
	Json::Value value;
	Json::Reader reader;
	if (!reader.parse(ifs, value, false))
	{
		throw("reader.parse Error");
	}
	ifs.close();
	return value;
}


//json list 轉 cv::Mat
cv::Mat json2cvMat(const Json::Value& jmat) {
	cv::Mat mat;
	if (jmat[0].type() != Json::ValueType::arrayValue) {
		mat= cv::Mat(1, jmat.size(), CV_64F);
		for (unsigned int i = 0; i < jmat.size(); i++) {
			mat.at<double>(0, i) = jmat[i].asDouble();
		}
	}
	else {
		mat = cv::Mat(jmat[0].size(), jmat.size(), CV_64F);
		for (unsigned int i = 0; i < jmat[0].size(); i++) {
			for (unsigned int j = 0; j < jmat.size(); j++) {
				mat.at<double>(i,j) = jmat[i][j].asDouble();
			}
		}
	}

	return mat;
}

//json list 轉 cv::Mat
Json::Value cvMat2Json(const cv::Mat& mat) {
	Json::Value jsonMat;
	if (mat.cols == 1) {
		for (int i = 0; i < mat.rows; i++) {
			jsonMat.append(mat.at<double>(i,0));
		}
	}
	else if (mat.rows == 1) {
		for (int i = 0; i < mat.cols; i++) {
			jsonMat.append(mat.at<double>(0,i));
		}
	}
	else {
		for (int i = 0; i < mat.rows; i++) {
			Json::Value jsonRow;
			for (int j = 0; j < mat.cols; j++) {
				jsonRow.append(mat.at<double>(i, j));
			}
			jsonMat.append(jsonRow);
		}
	}

	return jsonMat;
}
Json::Value markerPose2Json(std::unordered_map< MarkerId, cv::Affine3d>& markerPoses) {
	Json::Value jsonMarkersPoses;
	for (const auto& [id, pose] : markerPoses) {
		Json::Value jsonMarkerPoses;
		jsonMarkerPoses["id"] = id;
		cv::Mat rvec, tvec;
		std::tie(rvec, tvec) = T2RvecTvec(pose);
		jsonMarkerPoses["rvec"] = cvMat2Json(rvec);
		jsonMarkerPoses["tvec"] = cvMat2Json(tvec);
		jsonMarkersPoses.append(jsonMarkerPoses);
	}
	return jsonMarkersPoses;
}



std::unordered_map<int, cv::Affine3d> json2MarkersPose(const Json::Value& jmarkers) {
	std::unordered_map<int, cv::Affine3d> markers;
	for (const auto &jmarker : jmarkers) {
		cv::Affine3d pose(json2cvMat(jmarker["rvec"]),(json2cvMat(jmarker["tvec"])));
		markers.emplace(jmarker["id"].asInt(), pose);
	}
	return markers;
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






template<class Eigen,class CV>
inline CV eigenVec2CvPoint(const Eigen &point) {
	return CV(point.x(), point.y(), point.z());
}
template<class CV, class Eigen>
inline Eigen cvPoint2EigenVec(const CV &point) {
	return Eigen(point.x, point.y, point.z);
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


