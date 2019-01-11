//
// Created by lucas on 1/28/18.
//
#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

namespace ark{

    typedef pcl::PointXYZRGB PointType;

    /** Container for storing syncronized imu measurements */
    struct ImuPair{
        double timestamp;
        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
    };

    /** Container for storing measurement from a single sensor */
    enum class ImuMeasurementType {gyro,accel};
    struct ImuMeasurement{
      double timestamp;
      Eigen::Vector3d data;
      ImuMeasurementType type;
      bool operator<(const ImuMeasurement& right) const
        {
          return timestamp>right.timestamp;
        }
    };

    /** A frame containing an RGB image, a depth image, and a transformation matrix */
    class RGBDFrame {
    public:
        cv::Mat mTcw;
        cv::Mat imRGB;
        cv::Mat imDepth;
        int frameId;
        RGBDFrame(){
            mTcw = cv::Mat::eye(4,4,CV_32FC1);
            frameId = -1;
        }
        RGBDFrame(const RGBDFrame& frame)
        {
            frame.mTcw.copyTo(mTcw);
            frame.imRGB.copyTo(imRGB);
            frame.imDepth.copyTo(imDepth);
            frameId = frame.frameId;
        }

        typedef std::shared_ptr<RGBDFrame> Ptr;
    };

    /** A set of images taken on the same frame, possibly by multiple instruments/cameras */
    class MultiCameraFrame {
    public:
        /** Timestamp of the frame (may be -1 if not available) */
        double timestamp;

        /** Vector of images in the frame */
        std::vector<cv::Mat> images;

        /** Transformations matrices of the images (may be empty if transforms not available) */
        std::vector<cv::Mat> vecTcw;

        /** ID of the frame (may be -1 if not available) */
        int frameId;

        /** Construct an empty MultiCameraFrame with the given ID (default -1) */
        explicit MultiCameraFrame(int frame_id = -1){
            frameId = frame_id;
        }

        /** Construct a MultiCameraFrame from the given images,
          * transformation matrix, and frame ID*/
        MultiCameraFrame(std::vector<cv::Mat> images,
            std::vector<cv::Mat> vecTcw = std::vector<cv::Mat>(), int frame_id = -1){
            this->images = images;
            this->vecTcw = vecTcw;
            frameId = frame_id;
        }

        /** Construct a MultiCameraFrame from the given image,
          * transformation matrix, and frame ID */
        MultiCameraFrame(cv::Mat image, cv::Mat Tcw = cv::Mat(), int frame_id = -1){
            images.push_back(image);
            vecTcw.push_back(Tcw);
            frameId = frame_id;
        }

        /** Copy constructor for MultiCameraFrame */
        MultiCameraFrame(const MultiCameraFrame & frame) :
            images(frame.images.size()), vecTcw(frame.vecTcw.size()), frameId(frame.frameId)
        {   
            for (size_t i = 0; i < frame.vecTcw.size(); i++) {
                frame.vecTcw[i].copyTo(vecTcw[i]);
            }
            for (size_t i = 0; i < frame.images.size(); i++) {
                frame.images[i].copyTo(images[i]);
            }
        }

        /** Move constructor for MultiCameraFrame */
        MultiCameraFrame(const MultiCameraFrame && frame)
        {   
            images = std::move(frame.images);
            vecTcw = std::move(frame.vecTcw);
            frameId = std::move(frame.frameId);
        }

        /** Move assignment for MultiCameraFrame */
        MultiCameraFrame & operator=(MultiCameraFrame && other)
        {
            if (this != &other) {
                images = std::move(other.images);
                vecTcw = std::move(other.vecTcw);
                frameId = std::move(other.frameId);
            }
            return *this;
        }

        typedef std::shared_ptr<MultiCameraFrame> Ptr;
    };
}
