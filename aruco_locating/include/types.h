#ifndef _TYPES_H
#define _TYPES_H

#include "pch.h"

class Marker;

typedef int MarkerId;
typedef int FrameNumber;
typedef std::map<MarkerId, Marker> Markers;
typedef std::vector<cv::Point2f> PolyF;
typedef std::vector<cv::Point2i> Poly;
typedef std::vector<Poly> Polys;
typedef std::vector<PolyF> PolyFs;
typedef std::vector<cv::Point3d> Clouds;

enum class Source {
	VIDEO,
	IMAGE,
	LIVE_CAPTURE,
	ERROR
};
#endif