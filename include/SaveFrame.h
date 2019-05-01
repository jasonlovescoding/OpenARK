#ifndef OPENARK_SAVEFRAME_H
#define OPENARK_SAVEFRAME_H

#include "Types.h"

#include <mutex>
#include <thread>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>


namespace ark{

    class SaveFrame{
    public:
        SaveFrame(std::string folderPath);

        void OnKeyFrameAvailable(const RGBDFrame &keyFrame);

        void OnFrameAvailable(const RGBDFrame &frame);

        void frameWrite(const RGBDFrame &frame);

        ark::RGBDFrame frameLoad(int frameId);

    private:

        //Main Loop thread
        std::thread *mptRun;
        std::string folderPath;
        std::string rgbPath;
        std::string depthPath;
        std::string tcwPath;
        std::string depth_to_tcw_Path;


        //RGBDFrame Map
        std::map<int, ark::RGBDFrame> mMapRGBDFrame;

        //Current KeyFrame
        std::mutex mKeyFrameMutex;
        ark::RGBDFrame mKeyFrame;

        //Current Frame
        std::mutex mFrameMutex;
        ark::RGBDFrame mFrame;

        //Request Stop Status
        std::mutex mRequestStopMutex;
        bool mbRequestStop;

    };
}




#endif  //#define OPENARK_SAVEFRAME_H
