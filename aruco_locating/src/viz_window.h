#ifndef VIZWINDOW_H
#define VIZWINDOW_H


//��ı�Ƭɭ�
class VIZWindow {
public:
    //��skeyFrames,keyMarkers �쫺
    void updateWindow(const Frame& frame);
    //���
    bool isFreezeViz=false;
    //���
    bool isSnap = false;
    //�վ�]�w
    bool isAdjustSetting = false;
    //�������R
    bool isTerminate = false;
    bool isRescue = false;
    cv::viz::Viz3d operator()() {
        return _window;
    }
    //�i�ܵe��
    void show();
    ~VIZWindow();
private:
    cv::viz::Viz3d _window=cv::viz::Viz3d("Window");
    //������i�椬��(�ܹ�opencv ��waitKey(0))
    static void  keyboardViz3d(const cv::viz::KeyboardEvent& w, void* t);
};
#endif
