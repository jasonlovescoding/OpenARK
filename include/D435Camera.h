#pragma once
// OpenCV Libraries
#include "Version.h"
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>

// OpenARK Libraries
#include "DepthCamera.h"

namespace ark {
    /**
    * Class defining the behavior of a generic Intel RealSense Camera using RSSDK2.
    * Example on how to read from sensor and visualize its output
    * @include SensorIO.cpp
    */
    class D435Camera 
    {
    public:

        /**
        * Public constructor initializing the RealSense Camera.
        * @param use_rgb_stream if true, uses the RGB stream and disable the IR stream (which is on by default)
        *                       This results in a smaller field of view and has an appreciable performance cost.
        */
        explicit D435Camera();

        /**
        * Destructor for the RealSense Camera.
        */
        ~D435Camera();

        /**
         * Get the camera's model name.
         */
        const std::string getModelName() const;

        /** 
         * Returns the width of the SR300 camera frame 
         */
        int getWidth() const;

        /** 
         * Returns the height of the SR300 camera frame 
         */
        int getHeight() const;

        /** 
         * Sets the external hardware sync mode for the camera
         */
        void enableSync(bool flag);

        /**
         * Returns default detection parameters for this depth camera class
         */
        //const DetectionParams::Ptr & getDefaultParams() const ;

        /**
         * Returns true if an RGB image is available from this camera.
         * @return true if an RGB image is available from this camera.
         */
        //bool hasRGBMap() const override;

        /**
         * Returns true if an infrared (IR) image is available from this camera.
         * @return true if an infrared (IR) image is available from this camera.
         */
        //bool hasIRMap() const override;


        /** Preferred frame height */
        const int PREFERRED_FRAME_H = 480;

        /** Shared pointer to SR300 camera instance */
        typedef std::shared_ptr<D435Camera> Ptr;

        /**
        * Gets the new frame from the sensor (implements functionality).
        * Updates xyzMap and ir_map.
        */
        void update(MultiCameraFrame & frame);

    protected:

        /**
         * Initialize the camera, opening channels and resetting to initial configurations
         */
        void initCamera();

        /** Converts an D435 raw depth image to an ordered point cloud based on the current camera's intrinsics */
        void project(const rs2::frame & depth_frame, cv::Mat & xyz_map);

        /** Query RealSense camera intrinsics */
        //void query_intrinsics();

        std::shared_ptr<rs2::pipeline> pipe;
        rs2::config config;
        rs2::depth_sensor* depth_sensor;
        rs2::device device;
        rs2_intrinsics depthIntrinsics;

        double scale;
        int width, height;
        bool badInputFlag;
    };
}
