#include "stdafx.h"
#include "Version.h"
#include "SR300Camera.h"
#include "Visualizer.h"

using namespace Intel::RealSense;

namespace ark {
    /***
    Private constructor for the Intel RealSense SR300 camera depth sensor
    ***/
    SR300Camera::SR300Camera(bool use_rgb_stream) :
        dists(nullptr), amps(nullptr), depth_width(0), depth_height(0), sample(nullptr),
        useRGBStream(use_rgb_stream)
    {
        session->SetCoordinateSystem(CoordinateSystem::COORDINATE_SYSTEM_FRONT_DEFAULT);

        if (!sm)
        {
            wprintf_s(L"Unable to create the SenseManager!\n");
            return;
        }

        initCamera();
    }

    /***
    Destructor for the SR300 Camera depth sensor
    ***/
    SR300Camera::~SR300Camera() {
        printf("closing sensor\n");
        sm->Close();
        printf("sensor closed\n");
    };

    // overrided model name
    const std::string SR300Camera::getModelName() const
    {
        return "SR300";
    }

    // overrided width
    int SR300Camera::getWidth() const {
        // cut off 35 px to eliminate shadow on right
        return REAL_WID - 35;
    }

    // overrided height
    int SR300Camera::getHeight() const {
        return REAL_HI;
    }

    /**
    * true if has RGB image (override)
    */
    bool SR300Camera::hasRGBMap() const {
        return useRGBStream;
    }

    /**
    * true if has IR image (override)
    */
    bool SR300Camera::hasIRMap() const {
        return !useRGBStream;
    }

  /**
    * Create xyzMap, zMap, ampMap, and flagMap from sensor input (override)
    * @param [out] frame [0]:xyz_map(CV_32FC1) [1]:IR(CV_8UC1) / RGB(CV_8UC3)
    */
    void SR300Camera::update(MultiCameraFrame & frame)
    {
        Status sts = sm->AcquireFrame(true);
        if (sts < STATUS_NO_ERROR)
        {
            if (sts == Status::STATUS_STREAM_CONFIG_CHANGED)
            {
                wprintf_s(L"Stream configuration was changed, re-initializing\n");
                sm->ReleaseFrame();
                sm->Close();
                badInputFlag = true;
                return;
            }
        }

        // we always only use two images
        frame.images.resize(2);
        // init depth map, if needed
        if (frame.images[0].empty()) {
            frame.images[0] = cv::Mat(getImageSize(), CV_32FC3);
        }

        sample = sm->QuerySample();

        if (!sample || sample->depth == nullptr) {
            wprintf_s(L"Couldn't connect to camera, retrying in 0.5s...\n");
            sm->ReleaseFrame(); sm->Close();
            badInputFlag = true;
            boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
            initCamera();
            return;
        }

        badInputFlag = false;

        // get images from camera
        Image * depthSource = sample->depth;


    /**
    * Returns the X value at (i, j)
    */
//    float SR300Camera::getX(int i, int j) const
//    {
//        int flat = j * depth_width * 3 + i * 3;
//        return dists[flat];
//    }

    /**
    * Returns the Y value at (i, j)
    */
//    float SR300Camera::getY(int i, int j) const
//    {
//        int flat = j * depth_width * 3 + i * 3;
//        return dists[flat + 1];
//    }

    /**
    * Returns the Z value at (i, j)
    */
//    float SR300Camera::getZ(int i, int j) const
//    {
//        auto flat = j * depth_width * 3 + i * 3;
//        return dists[flat + 2];
//    }

    // Initialize camera (helper)
    void SR300Camera::initCamera() {
        if (!sm) return;
        cm = sm->QueryCaptureManager();
        auto sts = Status::STATUS_DATA_UNAVAILABLE;

        sm->EnableStream(Capture::STREAM_TYPE_DEPTH, REAL_WID, REAL_HI, depth_fps);
        sm->EnableStream(useRGBStream ? Capture::STREAM_TYPE_COLOR : Capture::STREAM_TYPE_IR,
                         REAL_WID, REAL_HI, depth_fps);

        sts = sm->Init();
        device = cm->QueryDevice();
    }
}
