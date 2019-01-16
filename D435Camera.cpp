#include "stdafx.h"
#include "Version.h"
#include "D435Camera.h"
#include "Visualizer.h"

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_pipeline.hpp>

/** RealSense SDK2 Cross-Platform Depth Camera Backend **/
namespace ark {
    D435Camera::D435Camera() {
        //Setup camera
        //TODO: Make read from config file
        width = 640;
        height = 480;
        config.enable_stream(RS2_STREAM_DEPTH,-1,width, height,RS2_FORMAT_Z16,30);
        config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, 30);
        config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, 30);

        //Start streaming data
        pipe = std::make_shared<rs2::pipeline>();
        rs2::pipeline_profile selection = pipe->start(config);

        //Disable IR emitter
        //Enable hardware syncronization
        rs2::device selected_device = selection.get_device();
        depth_sensor = new rs2::depth_sensor(selected_device.first<rs2::depth_sensor>());
        depth_sensor->set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
        depth_sensor->set_option(RS2_OPTION_INTER_CAM_SYNC_MODE,0);
    }

    D435Camera::~D435Camera() {
        try {
            pipe->stop();
            if(depth_sensor){
                delete depth_sensor;
                depth_sensor=nullptr;
            }
        }
        catch (...) {}
    }

    void D435Camera::enableSync(bool flag){
        depth_sensor->set_option(RS2_OPTION_INTER_CAM_SYNC_MODE,flag);
    }

    const std::string D435Camera::getModelName() const {
        return "RealSense";
    }

    int D435Camera::getWidth() const {
        return width;
    }

    int D435Camera::getHeight() const {
        return height;
    }

    /*const DetectionParams::Ptr & D435Camera::getDefaultParams() const {
        if (!defaultParamsSet) {
            defaultParamsSet = true;
            defaultParams = std::make_shared<DetectionParams>();
            defaultParams->contourImageErodeAmount = 0;
            defaultParams->contourImageDilateAmount = 2;
            defaultParams->fingerCurveFarMin = 0.18;
            defaultParams->fingerLenMin = 0.025;
            defaultParams->handClusterInterval = 15;
            defaultParams->handClusterMaxDistance = 0.003;
            defaultParams->handSVMConfidenceThresh = 0.52;
            defaultParams->handClusterMinPoints = 0.015;
            defaultParams->planeFloodFillThreshold = 0.19;
            defaultParams->planeEquationMinInliers = 0.02;
            defaultParams->planeMinPoints = 0.02;
            defaultParams->planeCombineThreshold = 0.0019;
            defaultParams->normalResolution = 3;
            defaultParams->handRequireEdgeConnected = false;
        }
        return defaultParams;
    }*/

    //bool D435Camera::hasRGBMap() const {
    //    return useRGBStream;
    //}

    //bool D435Camera::hasIRMap() const {
    //    return !useRGBStream;
    //}

    void D435Camera::update(MultiCameraFrame & frame) {

        try {
            // Ensure the frame has space for all images
            frame.images.resize(2);

            // Get frames from camera
            auto frames = pipe->wait_for_frames();
            auto infrared = frames.get_infrared_frame(1);        
            auto infrared2 = frames.get_infrared_frame(2);
            auto depth = frames.get_depth_frame();

            // Store ID for later
            frame.frameId = infrared.get_frame_number();

            // Convert infrared frame to opencv
            if (frame.images[0].empty()) frame.images[0] = cv::Mat(cv::Size(width,height), CV_8UC1);
            std::memcpy( frame.images[0].data, infrared.get_data(),width * height);

            if (frame.images[1].empty()) frame.images[1] = cv::Mat(cv::Size(width,height), CV_8UC1);
            std::memcpy( frame.images[1].data, infrared2.get_data(),width * height);

            //if (frame.images[2].empty()) frame.images[2] = cv::Mat(getImageSize(), CV_32FC3);
            //project(depth, frame.images[2]);


        } catch (std::runtime_error e) {
            // Try reconnecting
            badInputFlag = true;
            pipe->stop();
            printf("Couldn't connect to camera, retrying in 0.5s...\n");
            boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
            //query_intrinsics();
            pipe->start(config);
            badInputFlag = false;
            return;
        }
    }

    // project depth map to xyz coordinates directly (faster and minimizes distortion, but will not be aligned to RGB/IR)
    /*void D435Camera::project(const rs2::frame & depth_frame, cv::Mat & xyz_map) {
        const uint16_t * depth_data = (const uint16_t *)depth_frame.get_data();

        if (!depthIntrinsics || !rgbIntrinsics || !d2rExtrinsics) return;
        rs2_intrinsics * dIntrin = reinterpret_cast<rs2_intrinsics *>(depthIntrinsics);

        const uint16_t * srcPtr;
        cv::Vec3f * destPtr;
        float srcPixel[2], destXYZ[3];

        for (int r = 0; r < height; ++r)
        {
            srcPtr = depth_data + r * dIntrin->width;
            destPtr = xyz_map.ptr<Vec3f>(r);
            srcPixel[1] = r;

            for (int c = 0; c < width; ++c)
            {
                if (srcPtr[c] == 0) {
                    memset(&destPtr[c], 0, 3 * sizeof(float));
                    continue;
                }
                srcPixel[0] = c;
                rs2_deproject_pixel_to_point(destXYZ, dIntrin, srcPixel, srcPtr[c] * scale);
                memcpy(&destPtr[c], destXYZ, 3 * sizeof(float));
            }
        }
    }*/

    /*void D435Camera::query_intrinsics() {
        rs2_intrinsics * depthIntrinsics = new rs2_intrinsics();

        rs2::context ctx;
        rs2::device_list list = ctx.query_devices();

        ASSERT(list.size() > 0, "No camera detected.");
        const rs2::device & dev = list.front();
        const std::vector<rs2::sensor> sensors = dev.query_sensors();

        for (unsigned i = 0; i < sensors.size(); ++i) {
            const rs2::sensor & sensor = sensors[i];
            const std::vector<rs2::stream_profile> & stream_profiles = sensor.get_stream_profiles();

            for (unsigned j = 0; j < stream_profiles.size(); ++j) {
                const rs2::stream_profile & stream_profile = stream_profiles[j];
                const rs2_stream & stream_data_type = stream_profile.stream_type();
                const rs2_format & stream_format = stream_profile.format();

                if (stream_profile.is<rs2::video_stream_profile>()) {
                    if (stream_data_type == RS2_STREAM_DEPTH && stream_format == RS2_FORMAT_Z16) {
                        const rs2::video_stream_profile & prof = stream_profile.as<rs2::video_stream_profile>();
                        *depthIntrinsics = prof.get_intrinsics();
                        this->depthIntrinsics = depthIntrinsics;
                        if (depthIntrinsics->height == PREFERRED_FRAME_H) break;
                    }
                }
            }
            if (this->depthIntrinsics) break;
        }

        ASSERT(this->depthIntrinsics, "FATAL: Camera has no depth stream!");
        width = depthIntrinsics->width;
        height = depthIntrinsics->height;

        config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16);
        if (useRGBStream) config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8);
        else config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8);
    }*/
}
