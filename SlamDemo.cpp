#include "D435Camera.h"
#include "serial_sync_imu.h"
#include "OkvisSLAMSystem.h"
#include <iostream>
#include <thread>
#include "glfwManager.h"

using namespace ark;

int main(int argc, char **argv)
{

    if (argc != 4 && argc != 3 && argc !=2 ) {
        std::cerr << "Usage: ./" << argv[0] << " configuration-yaml-file [vocabulary-file] [skip-first-seconds]";
        return -1;
    }

    google::InitGoogleLogging(argv[0]);

    okvis::Duration deltaT(0.0);
    if (argc == 4) {
        deltaT = okvis::Duration(atof(argv[3]));
    }

    // read configuration file
    std::string configFilename;
    if (argc > 1) configFilename = argv[1];
    else configFilename = "intr.yml";

    std::string vocabFilename;
    if (argc > 2) vocabFilename = argv[2];
    else vocabFilename = "";

    OkvisSLAMSystem slam(vocabFilename, configFilename);

    //setup display
    if (!MyGUI::Manager::init())
    {
       fprintf(stdout, "Failed to initialize GLFW\n");
       return -1;
    }

    ImuParams params;
    params.rate = 200;
    params.gyro_div =  16.4 *180.0/M_PI;
    params.accel_div = 8192.0/9.81;
    SerialSyncImu imu0("COM4", &params);
    printf("Camera initialization started...\n");
    fflush(stdout);
    D435Camera camera;
    //printf("IMU initialization started...\n");
    //std::this_thread::sleep_for( std::chrono::duration<double, std::milli>(500));
    while(!imu0.start());
    //std::this_thread::sleep_for( std::chrono::duration<double, std::milli>(500));
    camera.start();

    printf("Camera-IMU initialization complete\n");
    fflush(stdout);

    //Window for displaying the path
    //PoseViewer poseViewer;
    MyGUI::CameraWindow path_win("Path Viewer", 1024, 620);
    MyGUI::ImageWindow img_win("Frame Viewer", 640,480, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    MyGUI::Path path1("path1", Eigen::Vector3d(1, 0, 0));
    MyGUI::Axis axis1("axis1", 1);
    MyGUI::Axis axis2("axis2", 1);
    MyGUI::Grid grid1("grid1", 10, 1);
    path_win.add_object(&path1);
    path_win.add_object(&axis1);
    path_win.add_object(&axis2);
    path_win.add_object(&grid1);

    //Recieves output from SLAM system and displays to the screen
    FrameAvailableHandler handler([&path1, &axis2, &img_win](MultiCameraFrame::Ptr frame) {
        Eigen::Affine3d transform(frame->T_WS());
        path1.add_node(transform.translation());
        axis2.set_transform(transform);
        img_win.set_image(frame->images_[0]);
    });
    slam.AddFrameAvailableHandler(handler, "mapping");

    LoopClosureDetectedHandler loopHandler([&slam, &path1](void) {
        std::vector<MapKeyFrame::Ptr> traj;
        slam.getTrajectory(traj);
        path1.clear();
        for(size_t i=0; i<traj.size(); i++){
            path1.add_node(traj[i]->T_WS().block<3,1>(0,3));
        }
    });
    slam.AddLoopClosureDetectedHandler(loopHandler,"trajectoryUpdate");
    //run until display is closed
    okvis::Time start(0.0);
    int id =0;
    while (MyGUI::Manager::running()){
        //printf("test\n");
        //Update the display
        MyGUI::Manager::update();

        //Get current camera frame
        MultiCameraFrame frame;
        camera.update(frame);

        //Get timestamp of camera
        //printf("getTs: %i\n",frame.frameId_);
        frame.timestamp_ = imu0.getFrameTimestamp(frame.frameId_);//
        //printf("frame ts: %f\n",frame.timestamp_/1e9);
        if(frame.timestamp_<0)
            continue;

        //Get or wait for IMU Data until current frame 
        std::vector<ImuPair> imuData;
        imu0.getImuToTime(frame.timestamp_,imuData);

        //Add data to SLAM system
        slam.PushIMU(imuData);
        slam.PushFrame(frame.images_, frame.timestamp_);

        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27) break; // 27 is ESC
    }
    printf("\nTerminate...\n");
    // Clean up
    slam.ShutDown();
    printf("\nExiting...\n");
    return 0;
}
