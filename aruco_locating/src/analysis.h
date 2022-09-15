#ifndef ANALYSIS_H
#define ANALYSIS_H
class Analysis {
public:
	//進行分析
	void run(const Source source, const std::string& sourcePath,const bool isParallel,const bool isMonitor,const bool isBA);
	//輸出分析結果
	void outputResults(const std::string& path) const;
private:
	//透過設定讓原本沒有偵測到的aruco標記可以被偵測到
	void rescue(Frame& frame);
	Source _source;
	//三維視覺視窗
	VIZWindow _window;
	//調整設定
	void adjustSetting(Frame& frame);
	//檢查使用者有下指令並做出相對應動作
	bool getCommand(Frame& frame);
	Record recorder;
	//儲存的上一幀
	Frame lastFrame;
	//分析一張照片
	bool analysis(const FrameNumber frameNumber, std::unique_ptr<cv::Mat> img, const std::string& imgName);
	//暫停分析
	void freezing(Frame& frame);
	//是否有要求平行處裡
	bool _isParallel = false;
	//是否有要求監督標記都有偵測到
	bool _isMonitor = false;
	//是否有要求開啟BA優化
	bool _isBA = false;
	//是否要求拍照
	bool _isSnap = false;
	//BA最佳化
	void BA(Frame& frame);

};

#endif // !ANALYSIS_H

