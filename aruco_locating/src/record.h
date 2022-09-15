#ifndef RECORD_H
#define RECORD_H

class Record  {
public:
	std::vector<Frame> frames;
	void addFrame(Frame& frame);
	void addFrame(Frame& frame, const bool isParallel);
	void output(const std::string& name) const;

private:

};

#endif

