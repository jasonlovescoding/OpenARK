#include "stdafx.h"
#include "Version.h"
#include "DepthCamera.h"
#include "Hand.h"
#include "FrameObject.h"

namespace ark {

    /**
     * Minimum depth of points (in meters). Points under this depth are presumed to be noise. (0.0 to disable)
     */
    const float DepthCamera::NOISE_FILTER_LOW = 0.14;

    /**
     * Maximum depth of points (in meters). Points above this depth are presumed to be noise. (0.0 to disable)
     */
    const float DepthCamera::NOISE_FILTER_HIGH = 0.99;

    DepthCamera::~DepthCamera()
    {
        badInputFlag = true;
        endCapture();
    }

    void DepthCamera::beginCapture(int fps_cap, bool remove_noise)
    {
        ASSERT(captureInterrupt == true, "beginCapture: already capturing from this camera");
        captureInterrupt = false;
        std::thread thd(&DepthCamera::captureThreadingHelper, this, fps_cap,
            &captureInterrupt, remove_noise);
        thd.detach();
    }

    void DepthCamera::endCapture()
    {
        captureInterrupt = true;
    }

    bool DepthCamera::nextFrame(bool removeNoise)
    {
        MultiCameraFrame::Ptr frameBuf =
            std::make_shared<MultiCameraFrame>(frame ? frame->frameId_ + 1 : 0);

        // update the backbuffer frame (to allow continued use of images on other threads)
        update(*frameBuf);

        // when update is done, move the backbuffer to front
        std::lock_guard<std::mutex> lock(imageMutex); // lock all buffers while moving
        frame = std::move(frameBuf);

        // call callbacks
        for (auto callback : updateCallbacks) {
            callback.second(*this);
        }

        return !badInput();

    }


    /** Returns default detection parameters for this depth camera class */
    const DetectionParams::Ptr & DepthCamera::getDefaultParams() const {
        return DetectionParams::DEFAULT;
    }

    /** Returns true on bad input */
    bool DepthCamera::badInput()
    {
        return badInputFlag;
    }

    /**
    Remove noise on zMap and xyzMap
    */
    void DepthCamera::removeNoise(cv::Mat & xyz_map, cv::Mat & amp_map, float confidence_thresh)
    {
        for (int r = 0; r < xyz_map.rows; ++r)
        {
            Vec3f * ptr = xyz_map.ptr<Vec3f>(r);

            const float * ampptr = nullptr;
            if (amp_map.data) ampptr = amp_map.ptr<float>(r);

            for (int c = 0; c < xyz_map.cols; ++c)
            {
                if (ptr[c][2] > 0.0f) {
                    if (ptr[c][2] < NOISE_FILTER_LOW &&
                        (ptr[c][2] > NOISE_FILTER_HIGH || ptr[c][2] == 0.0) &&
                        (ampptr == nullptr || amp_map.data == nullptr ||
                            ampptr[c] < confidence_thresh)) {
                        ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
                    }
                }
            }
        }
    }

    bool DepthCamera::isCapturing()
    {
        return !captureInterrupt;
    }

    int DepthCamera::addUpdateCallback(std::function<void(DepthCamera&)> func)
    {
        int id;
        if (updateCallbacks.empty()) {
            id = 0;
        }
        else {
            id = updateCallbacks.rbegin()->first + 1;
        }

        updateCallbacks[id] = func;
        return id;
    }

    void DepthCamera::removeUpdateCallback(int id)
    {
        updateCallbacks.erase(id);
    }

    cv::Size DepthCamera::getImageSize() const
    {
        return cv::Size(getWidth(), getHeight());
    }


    const MultiCameraFrame::Ptr DepthCamera::getFrame() const
    {
        return frame;
    }

    int DepthCamera::getFrameID() const
    {
        if (frame == nullptr) return 0;
        return frame->frameId_;
    }

    const std::string DepthCamera::getModelName() const {
        return "DepthCamera";
    }

    /**
    write a frame into file located at "destination"
    */
    bool DepthCamera::writeImage(std::string destination) const
    {
        cv::FileStorage fs(destination, cv::FileStorage::WRITE);
        std::lock_guard<std::mutex> lock(imageMutex);

        int N = (int) frame->images_.size();
        fs << "num_img" << N;
        for (int i = 0; i < N; ++i) {
            fs << ("img" + std::to_string(i)) << frame->images_[i];
        }

        fs.release();
        return true;
    }

    const cv::Mat DepthCamera::getFrameImage(int idx, int default_type) const
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        if (frame == nullptr || frame->images_.size() <= idx || frame->images_[idx].empty())
            return cv::Mat::zeros(getImageSize(), default_type);
        return frame->images_[idx];
    }

    /**
    Reads a frame from file located at "source"
    */
    bool DepthCamera::readImage(std::string source)
    {
        cv::FileStorage fs;
        fs.open(source, cv::FileStorage::READ);

        std::lock_guard<std::mutex> lock(imageMutex);
        int N; fs["num_img"] >> N;

        for (int i = 0; i < N; ++i) {
            cv::Mat m;
            fs["img" + std::to_string(i)] >> m;
            frame->images_.push_back(m);
        }
        fs.release();

        // call callbacks
        for (auto callback : updateCallbacks) {
            callback.second(*this);
        }

        return N > 0;
    }

    const cv::Mat DepthCamera::getXYZMap() const
    {
        return getFrameImage(2);
    }

    // note: depth camera must have XYZ map

    int DepthCamera::ampMapInvalidFlagValue() const{
        return -1;
    }

    float DepthCamera::flagMapConfidenceThreshold() const{
        return 0.5;
    }



    void DepthCamera::captureThreadingHelper(int fps_cap, volatile bool * interrupt, bool remove_noise)
    {
        using namespace std::chrono;
        steady_clock::time_point lastTime;
        steady_clock::time_point currTime;
        float timePerFrame;
        if (fps_cap > 0) {
            timePerFrame = 1e9f / fps_cap;
            lastTime = steady_clock::now();
        }

        while (interrupt == nullptr || !(*interrupt)) {
            this->nextFrame(remove_noise);

            // cap FPS
            if (fps_cap > 0) {
                currTime = steady_clock::now();
                steady_clock::duration delta = duration_cast<microseconds>(currTime - lastTime);

                if (delta.count() < timePerFrame) {
                    long long ms = (long long)(timePerFrame - delta.count()) / 1e6f;
                    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
                }
                lastTime = currTime;
            }
        }
    }
}
