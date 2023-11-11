#include "Videos/brainVideos.h"
#include "Videos/VideoInfos/yoruNiKakeru.h"
#include "Videos/VideoInfos/badApple.h"
#include "main.h"

void drawFrame(int x, int y, int width, int height, int frameId);
void switchVideoState();

std::vector< std::vector< std::vector<int> > > video;

int videoCount = 2;
double frameDelayMs;
int frameId;

void keybindVideos() {
    Controller1.ButtonDown.pressed([] () -> void {
        switchVideoState();
    });
}

void brainVideosThread() {
    frameId = 0;
    while (true) {
        if (playingVideoId > 0 && frameId >= 0) {
            drawFrame(0, 0, 480, 240, frameId);
            frameId++;
            frameId %= (int) video.size();
        }
        task::sleep(frameDelayMs);
    }
}

void drawFrame(int x, int y, int width, int height, int frameId) {
    if (frameId < 0 || frameId >= (int) video.size()) {
        return;
    }
    // Get frame
    printf("frame %d\n", frameId);
    std::vector< std::vector<int> > frame = video[frameId];
    // Draw frame at position
    int rgb;
    int posX, posY;
    // Loop through columns
    double iStep = (double) frame.size() / height;
    posY = y;
    for (double i = 0; i < (int) frame.size(); i += iStep) {
        int frameI = (int) i;
        // Loop through rows
        double jStep = (double) frame[frameI].size() / width;
        posX = x;
        for (double j = 0; j < (int) frame[frameI].size(); j += jStep) {
            int frameJ = (int) j;
            // Draw pixel
            rgb = frame[frameI][frameJ];
            if (rgb != -1) {
                Brain.Screen.setPenWidth(1);
                Brain.Screen.setPenColor(color(rgb));
                Brain.Screen.drawPixel(posX, posY);
            }
            // Update
            posX++;
        }
        posY++;
    }
}

bool videoDebounce = false;
void switchVideoState() {
    if (!videoDebounce) {
        videoDebounce = true;

        frameId = -10;
        // Increment video id
        playingVideoId++;
        playingVideoId %= (videoCount + 1);
        if (playingVideoId > 0) {
            printf("Playing video %d!\n", playingVideoId);
        }
        // Switch video
        switch (playingVideoId) {
            case 1:
                yoruNiKakeru.loadVideo(&video, &frameDelayMs);
                break;
            case 2:
                badApple.loadVideo(&video, &frameDelayMs);
                break;
            case 0:
                video.clear();
        }
        task::sleep(30);
        frameId = 0;
        task::sleep(30);

        videoDebounce = false;
    }
}