#ifndef RECORD_H
#define RECORD_H

#include "../pch.h"
#include "frame.h"

class Record  {
public:
	std::vector<Frame> frames;
	void addFrame(Frame& frame, const bool saveImg);
	void addFrame(Frame& frame, const bool saveImg, const bool isParallel);
	void output(const std::string& name) const;
	void outputFrames(const std::string& name) const;

private:

};

#endif

