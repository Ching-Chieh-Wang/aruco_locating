#ifndef _REGION_SELECT_H
#define _REGION_SELECT_H
#include "utils.h"
#include "types.h"

class RegionSelect {
private:
    cv::Mat _img;
    Polys _polys;
    std::stack<std::stack<cv::Point2i>> _polyRemoveHist;
    bool _isUserConfirmed = false;
    void showCurrent()const;
    static void mouseHandler(const int event, int x, int y, int flags, void* obj);
public:
    bool stop = false;
    Polys run(const cv::Mat& img);
};

#endif

