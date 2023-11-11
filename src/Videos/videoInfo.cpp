#include "Videos/videoInfo.h"
#include "main.h"

VideoInfo::VideoInfo() {
    fps = frameSteps = 0;
    savedVideo.clear();
}

VideoInfo::VideoInfo(double videoFps, double videoFrameSteps, std::vector< std::vector< std::vector<int> > > *video) {
    fps = videoFps;
    frameSteps = videoFrameSteps;
    savedVideo = *video;
}

void VideoInfo::saveVideo(std::vector< std::vector< std::vector<int> > > *video) {
    savedVideo = *video;
}

void VideoInfo::loadVideo(std::vector< std::vector< std::vector<int> > > *video, double *frameDelayMs) {
    *video = savedVideo;
    *frameDelayMs = (1.0 / fps) * (1000.0 / 1) * (frameSteps);
}