#include "region_select.h"
void RegionSelect::showCurrent() const{
    cv::Mat dispImg;
    _img.copyTo(dispImg);
    for (const Poly& poly : _polys) {
        if (poly.size() > 0) {
            cv::circle(dispImg, poly.front(), 5, cv::Scalar(255, 255, 0), 7, 8, 0);
            for (int i = 0; i < poly.size() - 1; i++) {
                line(dispImg, poly.at(i), poly.at(i + 1), cv::Scalar(0,255, 0),3,8,0);
            }
            cv::imshow("RegionSelect", dispImg);
        }

    }
    cv::imshow("RegionSelect", dispImg);
}




void RegionSelect::mouseHandler(int event, int x, int y, int flags, void* obj) {
    RegionSelect* maskCutter = static_cast<RegionSelect*>(obj);
    if (event == cv::EVENT_LBUTTONDOWN) {//畫範圍
        maskCutter->_isUserConfirmed = false;
        while (!maskCutter->_polyRemoveHist.empty()) maskCutter->_polyRemoveHist.pop();
        if (maskCutter->_polys.empty()) maskCutter->_polys.emplace_back(Poly());
        maskCutter->_polys.back().emplace_back(cv::Point2i(x, y));
        maskCutter->showCurrent();
    }
    if (event == cv::EVENT_RBUTTONDOWN) {//閉合
        if (!maskCutter->_polys.empty()){
            if (maskCutter->_polys.back().size() > 2) {
                mouseHandler(cv::EVENT_LBUTTONDOWN, maskCutter->_polys.back().front().x, maskCutter->_polys.back().front().y, 0, obj);
                maskCutter->_polys.emplace_back(Poly());
            }
        }
    }

    if (event == cv::EVENT_RBUTTONDBLCLK) { //確認
        maskCutter->_isUserConfirmed = true;
        if (maskCutter->_polys.empty()) return;
        if (maskCutter->_polys.back().size() <= 2) {
            maskCutter->_polys.pop_back();
        }
        maskCutter->showCurrent();
        cv::Mat mask (maskCutter->_img.size(), CV_8UC1, cv::Scalar::all(0));
        cv::Mat result(maskCutter->_img.size(), CV_8UC1, cv::Scalar::all(0));
        cv::fillPoly(mask, maskCutter->_polys, cv::Scalar(255, 255, 255), 8, 0);
        maskCutter->_polys.push_back(Poly{});
        bitwise_and(maskCutter->_img, maskCutter->_img, result, mask);
        cv::namedWindow("Mask", 0);
        cv::namedWindow("Result", 0);
        cv::resizeWindow("Mask", cv::Size(1500, 1500.f * maskCutter->_img.rows / maskCutter->_img.cols));
        cv::resizeWindow("Result", cv::Size(1500, 1500.f * maskCutter->_img.rows / maskCutter->_img.cols));
        cv::imshow("Mask", mask);
        cv::imshow("Result", result);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    if (event == cv::EVENT_MBUTTONDOWN){ //整個畫面
        while (!maskCutter->_polys.empty()) { //刪除原本
            mouseHandler(cv::EVENT_MOUSEWHEEL, -1, -1, 1, obj);
        }
        for(const cv::Point2i& corner: Frame::frameCorners()){
            mouseHandler(cv::EVENT_LBUTTONDOWN, corner.x, corner.y, 1, obj);
        }
        mouseHandler(cv::EVENT_RBUTTONDOWN, -1,-1, 1, obj);
        maskCutter->showCurrent();
    }
    if (event == cv::EVENT_MOUSEWHEEL) {
        if (flags > 0) { //scroll forward
            if (!maskCutter->_polys.empty()) {
                if (maskCutter->_polyRemoveHist.empty()) maskCutter->_polyRemoveHist.push(std::stack<cv::Point2i>());
                if (maskCutter->_polys.back().empty()) {
                    maskCutter->_polys.pop_back();
                    maskCutter->_polyRemoveHist.push(std::stack<cv::Point2i>());
                }
                maskCutter->_polyRemoveHist.top().push(maskCutter->_polys.back().back());
                maskCutter->_polys.back().pop_back();
                if (maskCutter->_polys.size() == 1 && maskCutter->_polys.front().empty()) maskCutter->_polys.pop_back();
            }
        }
        else { //scroll backward
            if (!maskCutter->_polyRemoveHist.empty()) {
                if (maskCutter->_polyRemoveHist.top().empty()) {
                    maskCutter->_polys.emplace_back(Poly());
                    maskCutter->_polyRemoveHist.pop();
                }
                if (maskCutter->_polys.empty()) maskCutter->_polys.emplace_back(Poly());
                maskCutter->_polys.back().emplace_back(maskCutter->_polyRemoveHist.top().top());
                maskCutter->_polyRemoveHist.top().pop();
                if (maskCutter->_polyRemoveHist.top().empty()) {
                    maskCutter->_polyRemoveHist.pop();
                    maskCutter->_polys.back().emplace_back(maskCutter->_polys.back().front());
                    maskCutter->_polys.emplace_back(Poly());
                }
                if (maskCutter->_polyRemoveHist.size() == 1 && maskCutter->_polyRemoveHist.top().empty()) maskCutter->_polyRemoveHist.pop();
            }
        }
        maskCutter->showCurrent();
    }
}


Polys RegionSelect::run(const cv::Mat& img) {
    img.copyTo(_img);
    for (const Poly& poly : Settings::detectRegions) {
        if (poly.size() > 0) {
            cv::circle(_img, poly.front(), 5, cv::Scalar(255, 0, 255),7, 8, 0);
            for (int i = 0; i < poly.size() - 1; i++) {
                line(_img, poly.at(i), poly.at(i + 1), cv::Scalar(0, 0, 255), 5, 8, 0);
            }
        }

    }
    cv::namedWindow("RegionSelect", 0);
    cv::resizeWindow("RegionSelect", cv::Size(1500, 1500.f * _img.rows / _img.cols));
    cv::setMouseCallback("RegionSelect", RegionSelect::mouseHandler, this);
    for(const Poly& poly:Settings::detectRegions)
    cv::imshow("RegionSelect", _img);
    cv::waitKey(0);
    cv::destroyAllWindows();
    if(!_polys.empty())_polys.pop_back();
    if (!_isUserConfirmed) _polys = Settings::detectRegions;
    return _polys;
}


