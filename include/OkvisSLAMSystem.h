#pragma once

#include "SLAMSystem.h"
#include <okvis/VioParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>
#include <thread>
#include <opencv2/core/eigen.hpp>
#include "SingleConsumerPriorityQueue.h"
#include <atomic>

namespace ark {
    /** Okvis-based SLAM system */
    class OkvisSLAMSystem : public SLAMSystem {

        struct StampedImages {
            std::vector<cv::Mat> images;
            okvis::Time timestamp;
            int id;
            bool operator<(const StampedImages& right) const
            {
                return timestamp > right.timestamp;
            }
        };

        struct StampedPath {
            std::vector<okvis::kinematics::Transformation> path;
            okvis::Time timestamp;
            bool loopClosureDetected;
            bool operator<(const StampedPath& right) const
            {
                return timestamp > right.timestamp;
            }
        };

        struct StampedState {
            okvis::kinematics::Transformation T_WS;
            okvis::Time timestamp;
            bool operator<(const StampedState& right) const
            {
                return timestamp > right.timestamp;
            }
        };

    public:

        OkvisSLAMSystem(const std::string &strVocFile, const std::string &strSettingsFile);

        void PushFrame(const std::vector<cv::Mat>& images, const double &timestamp);

        void PushFrame(const cv::Mat image, const double &timestamp);

        void PushIMU(const std::vector<ImuPair>& imu);

        void PushIMU(const ImuPair& imu);

        void PushIMU(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d gyro);

        void Start();

        void RequestStop();

        void ShutDown();

        bool IsRunning();

        void display();

        ~OkvisSLAMSystem();

        
        std::shared_ptr<okvis::ThreadedKFVio> okvis_estimator_;

    protected:
        void KeyFrameConsumerLoop();

        void FrameConsumerLoop();

    private:
        okvis::Time start_;
        okvis::Time t_imu_;
        okvis::Duration deltaT_;
        okvis::VioParameters parameters_;
        SingleConsumerPriorityQueue<StampedImages> image_queue_;
        SingleConsumerPriorityQueue<StampedImages> frame_queue_;
        SingleConsumerPriorityQueue<StampedPath> path_queue_;
        SingleConsumerPriorityQueue<StampedState> state_queue_;
        std::thread keyFrameConsumerThread_;
        std::thread frameConsumerThread_;
        int num_frames_;
        std::atomic<bool> kill;

    }; // OkvisSLAMSystem

}//ark
