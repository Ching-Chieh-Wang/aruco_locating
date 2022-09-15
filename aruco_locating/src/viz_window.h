#ifndef VIZWINDOW_H
#define VIZWINDOW_H


//視覺化界面
class VIZWindow {
public:
    //更新keyFrames,keyMarkers 位姿
    void updateWindow(const Frame& frame);
    //凍住
    bool isFreezeViz=false;
    //拍照
    bool isSnap = false;
    //調整設定
    bool isAdjustSetting = false;
    //結束分析
    bool isTerminate = false;
    bool isRescue = false;
    cv::viz::Viz3d operator()() {
        return _window;
    }
    //展示畫面
    void show();
    ~VIZWindow();
private:
    cv::viz::Viz3d _window=cv::viz::Viz3d("Window");
    //按按鍵進行互動(很像opencv 的waitKey(0))
    static void  keyboardViz3d(const cv::viz::KeyboardEvent& w, void* t);
};
#endif
