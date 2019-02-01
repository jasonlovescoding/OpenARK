#include "stdafx.h"
#include "OkvisSLAMSystem.h"

namespace ark {

    OkvisSLAMSystem::OkvisSLAMSystem(const std::string & strVocFile, const std::string & strSettingsFile) :
        start_(0.0), t_imu_(0.0), deltaT_(1.0), num_frames_(0), kill(false), 
        sparseMap_(true,strVocFile,true,new brisk::BruteForceMatcher()),cameraSystem_(new MultiCameraSystem()) {

        okvis::VioParametersReader vio_parameters_reader;
        try {
            vio_parameters_reader.readConfigFile(strSettingsFile);
        }
        catch (okvis::VioParametersReader::Exception ex) {
            std::cerr << ex.what() << "\n";
            return;
        }

        //okvis::VioParameters parameters;
        vio_parameters_reader.getParameters(parameters_);

        //create multicamera system from parameters
        //cameraSystem_.reset(new MultiCameraSystem());
        /*okvis::kinematics::Transformation T_SC;
        Eigen::VectorXd intr=Eigen::Matrix<double, 8, 1>::Identity(); //This should not be necessary, seems like Eigen bug? 
        for(size_t i =0; i< parameters_.nCameraSystem.numCameras(); i++){
            T_SC = *(parameters_.nCameraSystem.T_SC(i));
            parameters_.nCameraSystem.cameraGeometry(i)->getIntrinsics(intr);
            cameraSystem_->T_SC.push_back(T_SC.T());
            CameraCalibration cam_calib(intr[0],intr[1],intr[2],intr[3]);            
            //NOTE: assuming 4 parameter, pinhole distortion, need to add check 
            cam_calib.setDistortionCoefficients(intr[4],intr[5],intr[6],intr[7]);
            cameraSystem_->camera_calibrations.push_back(cam_calib);

        }
        for(size_t i =0; i< parameters_.secondaryCameraSystem.numCameras(); i++){
            T_SC = *(parameters_.secondaryCameraSystem.T_SC(i));
            parameters_.secondaryCameraSystem.cameraGeometry(i)->getIntrinsics(intr);
            cameraSystem_->T_SC.push_back(T_SC.T());
            CameraCalibration cam_calib(intr[0],intr[1],intr[2],intr[3]);            
            //NOTE: assuming 4 parameter, pinhole distortion, need to add check 
            cam_calib.setDistortionCoefficients(intr[4],intr[5],intr[6],intr[7]);
            cameraSystem_->camera_calibrations.push_back(cam_calib);

        }*/

        //initialize Visual odometry
        okvis_estimator_ = std::make_shared<okvis::ThreadedKFVio>(parameters_);
        okvis_estimator_->setBlocking(false);

        //register callback
        /*auto path_callback = [this](const okvis::Time& timestamp, const std::vector<okvis::kinematics::Transformation> & path, bool loopClosure) {
            path_queue_.enqueue({ path, timestamp, loopClosure });
        };

        auto state_callback = [this](const okvis::Time& timestamp, const okvis::kinematics::Transformation& T_WS) {
            state_queue_.enqueue({ T_WS,timestamp });
            //std::cout << "state callback: " << timestamp << std::endl;
        };*/

        //Okvis's outframe is our inframe
        auto frame_callback = [this](const okvis::Time& timestamp, okvis::OutFrameData::Ptr frame_data) {
            frame_data_queue_.enqueue({ frame_data,timestamp });
        };

        /*okvis_estimator_->setPathCallback(
            path_callback);

        okvis_estimator_->setStateCallback(
            state_callback);*/

        okvis_estimator_->setFrameCallback(frame_callback);

        //at thread to pull from queue and call our own callbacks
        //keyFrameConsumerThread_ = std::thread(&OkvisSLAMSystem::KeyFrameConsumerLoop, this);
        frameConsumerThread_ = std::thread(&OkvisSLAMSystem::FrameConsumerLoop, this);

        //path_queue_.clear();
        //state_queue_.clear();
        frame_data_queue_.clear();
        frame_queue_.clear();
        std::cout << "SLAM Initialized\n";
        //image_queue_.clear();
    }

    void OkvisSLAMSystem::Start() {
        //Do nothing, starts automatically
    }

    /*void OkvisSLAMSystem::KeyFrameConsumerLoop() {
        while (true) {
            StampedPath path;
            while (!path_queue_.try_dequeue(&path)) {
                if(kill)
                    return;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // get the most up to date frame
            StampedImages frame;
            bool frame_found = false;
            do {
                if (!(frame_found = image_queue_.try_dequeue(&frame))) {
                    break;
                }
            } while (frame.timestamp < path.timestamp);
            if (!frame_found)
                continue;
            MultiCameraFrame out_frame;
            out_frame.images = frame.images;
            out_frame.frameId = frame.id;
            okvis::kinematics::Transformation T_WS = path.path[path.path.size() - 1];
            for (size_t i = 0; i < frame.images.size(); i++) {
                okvis::kinematics::Transformation T_CW;
                if (i < parameters_.nCameraSystem.numCameras()) {
                    T_CW = (T_WS*(*parameters_.nCameraSystem.T_SC(i))).inverse();
                }
                else if (i - parameters_.nCameraSystem.numCameras() < parameters_.secondaryCameraSystem.numCameras()) {
                    T_CW = (T_WS*(*parameters_.secondaryCameraSystem.T_SC(i - parameters_.nCameraSystem.numCameras()))).inverse();
                }
                else {
                    //no relationship data to imu
                    //could potentially also set this to imu position
                    //or throw error
                    T_CW = okvis::kinematics::Transformation::Identity();
                }

                //convert transform to Mat
                cv::Mat cvT_CW;
                cv::eigen2cv(T_CW.T(), cvT_CW);
                out_frame.vecTcw.push_back(cvT_CW);
                //for each callback
                for (MapKeyFrameAvailableHandler::const_iterator callback_iter = mMapKeyFrameAvailableHandler.begin();
                    callback_iter != mMapKeyFrameAvailableHandler.end(); ++callback_iter) {
                    const MapKeyFrameAvailableHandler::value_type& pair = *callback_iter;
                    pair.second(out_frame);
                }

                //for each loop closure callback
                if (path.loopClosureDetected)
                    for (MapLoopClosureDetectedHandler::const_iterator callback_iter = mMapLoopClosureHandler.begin();
                        callback_iter != mMapLoopClosureHandler.end(); ++callback_iter) {
                    const MapLoopClosureDetectedHandler::value_type& pair = *callback_iter;
                    pair.second();
                }
            }

        }

    }*/

/*    void OkvisSLAMSystem::FrameConsumerLoop() {
        while (true) {
            //switch this to frame and then wait for state near frame
            StampedState state;
            while (!state_queue_.try_dequeue(&state)) {
                if(kill)
                    return;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            //get most up to date frame
            StampedImages frame;
            bool frame_found = false;
            //std::cout << "sz: " << state_queue_.size() << std::endl;
            do {
                if (!(frame_found = frame_queue_.try_dequeue(&frame))) {
                    break;
                }
                //std::cout << "frame ts: " << frame.timestamp << " state ts: " << state.timestamp << "sz: " << frame_queue_.size() << std::endl; 
            } while (frame.timestamp < state.timestamp);
            if (!frame_found)
                continue;
            MultiCameraFrame out_frame;
            out_frame.images = frame.images;
            out_frame.frameId = frame.id;
            for (size_t i = 0; i < frame.images.size(); i++) {
                okvis::kinematics::Transformation T_CW;
                if (i < parameters_.nCameraSystem.numCameras()) {
                    T_CW = (state.T_WS*(*parameters_.nCameraSystem.T_SC(i))).inverse();
                }
                else if (i - parameters_.nCameraSystem.numCameras() < parameters_.secondaryCameraSystem.numCameras()) {
                    T_CW = (state.T_WS*(*parameters_.secondaryCameraSystem.T_SC(i - parameters_.nCameraSystem.numCameras()))).inverse();
                }
                else {
                    //no relationship data to imu
                    //could potentially also set this to imu position
                    //or throw error
                    T_CW = okvis::kinematics::Transformation::Identity();
                }

                //convert transform to Mat
                cv::Mat cvT_CW;
                cv::eigen2cv(T_CW.T(), cvT_CW);
                out_frame.vecTcw.push_back(cvT_CW);
                //for each callback
                for (MapFrameAvailableHandler::const_iterator callback_iter = mMapFrameAvailableHandler.begin();
                    callback_iter != mMapFrameAvailableHandler.end(); ++callback_iter) {
                    const MapFrameAvailableHandler::value_type& pair = *callback_iter;
                    pair.second(out_frame);
                }
            }
        }
    }*/

    void OkvisSLAMSystem::FrameConsumerLoop() {
        while (true) {
            //Get processed frame data from OKVIS
            StampedFrameData frame_data;
            while (!frame_data_queue_.try_dequeue(&frame_data)) {
                if(kill)
                    return;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            //Get the corresponding frame from queue
            StampedImages frame;
            bool frame_found = false;
            do {
                if (!(frame_found = frame_queue_.try_dequeue(&frame))) {
                    break;
                } 
            } while (frame.timestamp < frame_data.timestamp);
            if (!frame_found){
                std::cout << "ERROR, FRAME NOT FOUND, THIS SHOULDN'T HAPPEN\n";
                continue;
            }

            //construct output frame
            MultiCameraFrame::Ptr out_frame = MultiCameraFrame::Ptr(new MultiCameraFrame);
            out_frame->images_ = frame.images;
            out_frame->frameId_ = frame_data.data->id;
            out_frame->T_KS_ = frame_data.data->T_KS.T();
            out_frame->T_WS_ = frame_data.data->T_WS.T();
            out_frame->keyframeId_ = frame_data.data->keyframe_id;
            out_frame->keyframe_ = sparseMap_.getKeyframe(out_frame->keyframeId_);
            out_frame->isKeyframe_ = frame_data.data->is_keyframe;

            //add sensor transforms
            //Note: this could potentially just be done once for the system
            //Including here for now in case we switch to optimized results
            /*for (size_t i = 0; i < frame.images.size(); i++) {
                okvis::kinematics::Transformation T_SC;
                if (i < parameters_.nCameraSystem.numCameras()) {
                    T_SC = (*parameters_.nCameraSystem.T_SC(i));
                }
                else if (i - parameters_.nCameraSystem.numCameras() < parameters_.secondaryCameraSystem.numCameras()) {
                    T_SC = (*parameters_.secondaryCameraSystem.T_SC(i - parameters_.nCameraSystem.numCameras()));
                } else {
                    //no relationship data to imu
                    //TODO: throw errror
                    T_SC = okvis::kinematics::Transformation::Identity();
                }
                out_frame->T_SC_.push_back(T_SC.T());
            }*/
            //out_frame->cameraSystem_=cameraSystem_;

            //check if keyframe
            if(out_frame->isKeyframe_){
                if(out_frame->keyframeId_!=out_frame->frameId_){
                    std::cout << "ERROR, KEYFRAME ID INCORRECT, THIS SHOULDN'T HAPPEN\n";
                    continue;
                }
                //copy keypoints and descriptors to output
                out_frame->keypoints_.resize(frame_data.data->keypoints.size());
                out_frame->descriptors_.resize(frame_data.data->descriptors.size());
                for(size_t cam_idx=0 ; cam_idx<frame_data.data->keypoints.size() ; cam_idx++){
                    out_frame->keypoints_[cam_idx].resize(frame_data.data->keypoints[cam_idx].size());
                    for(int i=0; i<frame_data.data->keypoints[cam_idx].size(); i++){
                        out_frame->keypoints_[cam_idx][i] = frame_data.data->keypoints[cam_idx][i];
                    }
                    out_frame->descriptors_[cam_idx]=frame_data.data->descriptors[cam_idx];
                }

                // push to map
                // may still need to add 3d points in camera coordinates
                if(sparseMap_.addKeyframe(out_frame)){ //add keyframe returns true if a loop closure was detected
                    for (MapLoopClosureDetectedHandler::const_iterator callback_iter = mMapLoopClosureHandler.begin();
                        callback_iter != mMapLoopClosureHandler.end(); ++callback_iter) {
                        const MapLoopClosureDetectedHandler::value_type& pair = *callback_iter;
                            pair.second();
                    }
                }
            }


            //Notify callbacks
            for (MapFrameAvailableHandler::const_iterator callback_iter = mMapFrameAvailableHandler.begin();
                callback_iter != mMapFrameAvailableHandler.end(); ++callback_iter) {
                const MapFrameAvailableHandler::value_type& pair = *callback_iter;
                pair.second(out_frame);
            }
        }
    }

    void OkvisSLAMSystem::PushFrame(const std::vector<cv::Mat>& images, const double &timestamp) {
        if (okvis_estimator_ == nullptr)
            return;
        okvis::Time t_image(timestamp / 1e9);
        if (start_ == okvis::Time(0.0)) {
            start_ = t_image;
        }

        if (t_image - start_ > deltaT_) {
            //right now we are keeping two queues, one for frame callback and one for keyframe callback
            //will change to be more efficient later
            /*if(mMapKeyFrameAvailableHandler.size()>0){
                image_queue_.enqueue({ images,t_image,num_frames_ });
                std::cout << "enqueue: " << t_image << std::endl;
            }*/
            if(mMapFrameAvailableHandler.size()>0){
                frame_queue_.enqueue({ images,t_image,num_frames_ });
            }
            num_frames_++;
            for (size_t i = 0; i < images.size(); i++) {
                if (i < parameters_.nCameraSystem.numCameras())
                    //printf("add image: %i\n", i);
                    okvis_estimator_->addImage(t_image, i, images[i]);
            }
        }
    }

    void OkvisSLAMSystem::PushFrame(const cv::Mat image, const double &timestamp) {
        if (okvis_estimator_ == nullptr)
            return;
        okvis::Time t_image(timestamp / 1e9);
        if (start_ == okvis::Time(0.0)) {
            start_ = t_image;
        }

        if (t_image - start_ > deltaT_) {
            std::vector<cv::Mat> images;
            images.push_back(image);
            /*if(mMapKeyFrameAvailableHandler.size()>0){
                image_queue_.enqueue({ images,t_image,num_frames_ });
                std::cout << "enqueue: " << t_image << std::endl;
            }*/
            if(mMapFrameAvailableHandler.size()>0){
                frame_queue_.enqueue({ images,t_image,num_frames_ });
            }
            num_frames_++;
            okvis_estimator_->addImage(t_image, 0, image);
        }
    }

    void OkvisSLAMSystem::PushIMU(const std::vector<ImuPair>& imu) {
        if (okvis_estimator_ == nullptr) return;
        for (size_t i = 0; i < imu.size(); i++) {
            PushIMU(imu[i]);
        }
    }

    void OkvisSLAMSystem::PushIMU(const ImuPair& imu) {
        if (okvis_estimator_ == nullptr) return;
        t_imu_ = okvis::Time(imu.timestamp / 1e9);
        if (t_imu_ - start_ + okvis::Duration(1.0) > deltaT_) {
            //std::cout << imu.timestamp / 1e9 << " , " << imu.accel.transpose() << " , " << imu.gyro.transpose() << std::endl;
            okvis_estimator_->addImuMeasurement(t_imu_, imu.accel, imu.gyro);
        }
    }

    void OkvisSLAMSystem::PushIMU(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d gyro) {
        if (okvis_estimator_ == nullptr) return;
        t_imu_ = okvis::Time(timestamp / 1e9);
        if (t_imu_ - start_ + okvis::Duration(1.0) > deltaT_) {
            okvis_estimator_->addImuMeasurement(t_imu_, accel, gyro);
        }
    }

    void OkvisSLAMSystem::display() {
        if (okvis_estimator_ == nullptr)
            return;
        okvis_estimator_->display();
    }

    void OkvisSLAMSystem::ShutDown()
    {
        //path_queue_.clear();
        //state_queue_.clear();
        frame_queue_.clear();
        frame_data_queue_.clear();
        //image_queue_.clear();
        kill=true;
        //keyFrameConsumerThread_.join();
        frameConsumerThread_.join();
        okvis_estimator_.reset();
    }

    OkvisSLAMSystem::~OkvisSLAMSystem() {
        //path_queue_.clear();
        //state_queue_.clear();
        frame_queue_.clear();
        frame_data_queue_.clear();
        //image_queue_.clear();
        kill=true;
    }

    void OkvisSLAMSystem::RequestStop()
    {
        okvis_estimator_ = nullptr;
    }

    bool OkvisSLAMSystem::IsRunning()
    {
        return okvis_estimator_ == nullptr;
    }

    void OkvisSLAMSystem::getTrajectory(std::vector<Eigen::Matrix4d>& trajOut){
        sparseMap_.getTrajectory(trajOut);
    }


} //ark