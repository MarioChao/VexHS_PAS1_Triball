#pragma once

#include <vector>

class VideoInfo {
    public:
        VideoInfo();
        VideoInfo(double fps, double frameSteps, std::vector< std::vector< std::vector<int> > > *video);
        void saveVideo(std::vector< std::vector< std::vector<int> > > *video);
        void loadVideo(std::vector< std::vector< std::vector<int> > > *video, double *frameDelayMs);
    private:
        double fps;
        double frameSteps;
        std::vector< std::vector< std::vector<int> > > savedVideo;
};