#include "D435Camera.h"
#include "serial_sync_imu.h"
#include "OkvisSLAMSystem.h"
#include <iostream>
#include <thread>
#include "glfwManager.h"

using namespace ark;


/*class PoseViewer
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MyGUI::Axis _axis = MyGUI::Axis("Axis2",1);
  MyGUI::Path _path3d= MyGUI::Path("Path1",Eigen::Vector3d(1, 0, 0));
  PoseViewer()
  {
    drawing_ = false;
    showing_ = false;
  }
  // this we can register as a callback
  void publishFullStateAsCallback(
      const okvis::Time & , const okvis::kinematics::Transformation & T_WS,
      const Eigen::Matrix<double, 9, 1> & speedAndBiases,
      const Eigen::Matrix<double, 3, 1> & )
  {

    // just append the path
    Eigen::Vector3d r = T_WS.r();
    Eigen::Matrix3d C = T_WS.C();

    // draw it
    while (showing_) {
    }
    drawing_ = true;

    _path3d.add_node(Eigen::Vector3d(r.x(),r.z(),-r.y()));
    _axis.set_transform(Eigen::Translation3d(Eigen::Vector3d(r.x(),r.z(),-r.y()))*Eigen::Quaterniond(C));

    drawing_ = false; // notify
  }
  void display()
  {
    while (drawing_) {
    }
    showing_ = true;
    MyGUI::Manager::update();
    //cv::imshow("OKVIS Top View", _image);
    showing_ = false;
    //cv::waitKey(1);
  }
 private:

  std::atomic_bool drawing_;
  std::atomic_bool showing_;
};*/


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
    SerialSyncImu imu0("/dev/cu.usbmodem1412101", &params);
    std::this_thread::sleep_for( std::chrono::duration<double, std::milli>(100));
    printf("Camera initialization started...\n");
    D435Camera camera;
    imu0.start();
    camera.enableSync(true);

    printf("Camera initialization complete\n");
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
    //path_win.add_object(&(poseViewer._axis));
    //path_win.add_object(&(poseViewer._path3d));
    //slam.okvis_estimator_->setFullStateCallback(
    //  std::bind(&PoseViewer::publishFullStateAsCallback, &poseViewer,
    //            std::placeholders::_1, std::placeholders::_2,
    //            std::placeholders::_3, std::placeholders::_4));

    //Recieves output from SLAM system and displays to the screen
    FrameAvailableHandler handler([&path1, &axis2, &img_win](MultiCameraFrame frame) {
        Eigen::Matrix4d eigen_Tcw;
        cv::cv2eigen(frame.vecTcw[0],eigen_Tcw);
        Eigen::Affine3d transform(eigen_Tcw.inverse());
        path1.add_node(transform.translation());
        axis2.set_transform(transform);
        img_win.set_image(frame.images[0]);
    });
    slam.AddFrameAvailableHandler(handler, "mapping");

    //run until display is closed
    okvis::Time start(0.0);
    int id =0;
    while (MyGUI::Manager::running()){
        printf("test\n");
        //Update the display
        //poseViewer.display();
        MyGUI::Manager::update();

        //Get current camera frame
        MultiCameraFrame frame;
        camera.update(frame);

        //Get timestamp of camera
        frame.timestamp = imu0.getFrameTimestamp(frame.frameId);//
        printf("frame ts: %f\n",frame.timestamp/1e9);

        //Get or wait for IMU Data until current frame 
        std::vector<ImuPair> imuData;
        imu0.getImuToTime(frame.timestamp,imuData);

        //Add data to SLAM system
        slam.PushIMU(imuData);
        slam.PushFrame(frame.images, frame.timestamp);

        //cv::imshow("Img 0", frame.images[0]);
        //cv::imshow("Img 1", frame.images[1]);

        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27) break; // 27 is ESC
    }
    printf("\nTerminate...\n");
    // Clean up
    //slam.ShutDown();
    printf("\nExiting...\n");
    return 0;
}
