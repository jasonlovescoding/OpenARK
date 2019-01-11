#include "D435Camera.h"
#include "serial_sync_imu.h"
#include "OkvisSLAMSystem.h"
#include <iostream>
#include <thread>
//#include "glfwManager.h"

using namespace ark;

class PoseViewer
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  constexpr static const double imageSize = 500.0;
  PoseViewer()
  {
    cv::namedWindow("OKVIS Top View");
    cv::namedWindow("PoseGraph Top View");

    _image.create(imageSize, imageSize, CV_8UC3);
    _image2.create(imageSize, imageSize, CV_8UC3);
    drawing_ = false;
    showing_ = false;
  }
  // this we can register as a callback
  void publishFullStateAsCallback(
      const okvis::Time & /*t*/, const okvis::kinematics::Transformation & T_WS,
      const Eigen::Matrix<double, 9, 1> & speedAndBiases,
      const Eigen::Matrix<double, 3, 1> & /*omega_S*/)
  {

    // just append the path
    Eigen::Vector3d r = T_WS.r();
    Eigen::Matrix3d C = T_WS.C();
    _path.push_back(cv::Point2d(r[0], r[1]));
    _heights.push_back(r[2]);
    // maintain scaling
    if (r[0] - _frameScale < _min_x)
      _min_x = r[0] - _frameScale;
    if (r[1] - _frameScale < _min_y)
      _min_y = r[1] - _frameScale;
    if (r[2] < _min_z)
      _min_z = r[2];
    if (r[0] + _frameScale > _max_x)
      _max_x = r[0] + _frameScale;
    if (r[1] + _frameScale > _max_y)
      _max_y = r[1] + _frameScale;
    if (r[2] > _max_z)
      _max_z = r[2];
    _scale = std::min(imageSize / (_max_x - _min_x), imageSize / (_max_y - _min_y));

    // draw it
    while (showing_) {
    }
    drawing_ = true;
    // erase
    _image.setTo(cv::Scalar(10, 10, 10));
    drawPath();
    // draw axes
    Eigen::Vector3d e_x = C.col(0);
    Eigen::Vector3d e_y = C.col(1);
    Eigen::Vector3d e_z = C.col(2);
    cv::line(
        _image,
        convertToImageCoordinates(_path.back()),
        convertToImageCoordinates(
            _path.back() + cv::Point2d(e_x[0], e_x[1]) * _frameScale),
        cv::Scalar(0, 0, 255), 1, CV_AA);
    cv::line(
        _image,
        convertToImageCoordinates(_path.back()),
        convertToImageCoordinates(
            _path.back() + cv::Point2d(e_y[0], e_y[1]) * _frameScale),
        cv::Scalar(0, 255, 0), 1, CV_AA);
    cv::line(
        _image,
        convertToImageCoordinates(_path.back()),
        convertToImageCoordinates(
            _path.back() + cv::Point2d(e_z[0], e_z[1]) * _frameScale),
        cv::Scalar(255, 0, 0), 1, CV_AA);

    // some text:
    std::stringstream postext;
    postext << "position = [" << r[0] << ", " << r[1] << ", " << r[2] << "]";
    cv::putText(_image, postext.str(), cv::Point(15,15),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1);
    std::stringstream veltext;
    veltext << "velocity = [" << speedAndBiases[0] << ", " << speedAndBiases[1] << ", " << speedAndBiases[2] << "]";
    cv::putText(_image, veltext.str(), cv::Point(15,35),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1);

    drawing_ = false; // notify
  }

  void publishStateAsCallback(
        const okvis::Time & /*t*/, const okvis::kinematics::Transformation & T_WS)
    {

      // just append the path
      Eigen::Vector3d r = T_WS.r();
      Eigen::Matrix3d C = T_WS.C();
      _path2.push_back(cv::Point2d(r[0], r[1]));
      _heights2.push_back(r[2]);
      // maintain scaling
      if (r[0] - _frameScale < _min_x2)
        _min_x2 = r[0] - _frameScale;
      if (r[1] - _frameScale < _min_y2)
        _min_y2 = r[1] - _frameScale;
      if (r[2] < _min_z2)
        _min_z2 = r[2];
      if (r[0] + _frameScale > _max_x2)
        _max_x2 = r[0] + _frameScale;
      if (r[1] + _frameScale > _max_y2)
        _max_y2 = r[1] + _frameScale;
      if (r[2] > _max_z2)
        _max_z2 = r[2];
      _scale2 = std::min(imageSize / (_max_x2 - _min_x2), imageSize / (_max_y2 - _min_y2));

      // draw it
      while (showing_) {
      }
      drawing_ = true;
      // erase
      _image2.setTo(cv::Scalar(10, 10, 10));
      drawPath2();
      // draw axes
      Eigen::Vector3d e_x = C.col(0);
      Eigen::Vector3d e_y = C.col(1);
      Eigen::Vector3d e_z = C.col(2);
      cv::line(
          _image2,
          convertToImageCoordinates2(_path2.back()),
          convertToImageCoordinates2(
              _path2.back() + cv::Point2d(e_x[0], e_x[1]) * _frameScale),
          cv::Scalar(0, 0, 255), 1, CV_AA);
      cv::line(
          _image2,
          convertToImageCoordinates2(_path2.back()),
          convertToImageCoordinates2(
              _path2.back() + cv::Point2d(e_y[0], e_y[1]) * _frameScale),
          cv::Scalar(0, 255, 0), 1, CV_AA);
      cv::line(
          _image2,
          convertToImageCoordinates2(_path2.back()),
          convertToImageCoordinates2(
              _path2.back() + cv::Point2d(e_z[0], e_z[1]) * _frameScale),
          cv::Scalar(255, 0, 0), 1, CV_AA);

      // some text:
      std::stringstream postext;
      postext << "position = [" << r[0] << ", " << r[1] << ", " << r[2] << "]";
      cv::putText(_image2, postext.str(), cv::Point(15,15),
                  cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1);

      drawing_ = false; // notify
    }

  void display()
  {
    while (drawing_) {
    }
    showing_ = true;
    cv::imshow("OKVIS Top View", _image);
    cv::imshow("PoseGraph Top View", _image2);
    showing_ = false;
    cv::waitKey(1);
  }
 private:
  cv::Point2d convertToImageCoordinates(const cv::Point2d & pointInMeters) const
  {
    cv::Point2d pt = (pointInMeters - cv::Point2d(_min_x, _min_y)) * _scale;
    return cv::Point2d(pt.x, imageSize - pt.y); // reverse y for more intuitive top-down plot
  }
  cv::Point2d convertToImageCoordinates2(const cv::Point2d & pointInMeters) const
  {
    cv::Point2d pt = (pointInMeters - cv::Point2d(_min_x2, _min_y2)) * _scale2;
    return cv::Point2d(pt.x, imageSize - pt.y); // reverse y for more intuitive top-down plot
  }
  void drawPath()
  {
    for (size_t i = 0; i + 1 < _path.size(); ) {
      cv::Point2d p0 = convertToImageCoordinates(_path[i]);
      cv::Point2d p1 = convertToImageCoordinates(_path[i + 1]);
      cv::Point2d diff = p1-p0;
      if(diff.dot(diff)<2.0){
        _path.erase(_path.begin() + i + 1);  // clean short segment
        _heights.erase(_heights.begin() + i + 1);
        continue;
      }
      double rel_height = (_heights[i] - _min_z + _heights[i + 1] - _min_z)
                      * 0.5 / (_max_z - _min_z);
      cv::line(
          _image,
          p0,
          p1,
          rel_height * cv::Scalar(255, 0, 0)
              + (1.0 - rel_height) * cv::Scalar(0, 0, 255),
          1, CV_AA);
      i++;
    }
  } 

  void drawPath2()  
  {
    for (size_t i = 0; i + 1 < _path2.size(); ) {
      cv::Point2d p0 = convertToImageCoordinates2(_path2[i]);
      cv::Point2d p1 = convertToImageCoordinates2(_path2[i + 1]);
      cv::Point2d diff = p1-p0;
      if(diff.dot(diff)<2.0){
        _path2.erase(_path2.begin() + i + 1);  // clean short segment
        _heights2.erase(_heights2.begin() + i + 1);
        continue;
      }
      double rel_height = (_heights2[i] - _min_z2 + _heights2[i + 1] - _min_z2)
                      * 0.5 / (_max_z2 - _min_z2);
      cv::line(
          _image2,
          p0,
          p1,
          rel_height * cv::Scalar(255, 0, 0)
              + (1.0 - rel_height) * cv::Scalar(0, 0, 255),
          1, CV_AA);
      i++;
    }
  }

  cv::Mat _image;
  std::vector<cv::Point2d> _path;
  std::vector<double> _heights;
  double _scale = 1.0;
  double _min_x = -0.5;
  double _min_y = -0.5;
  double _min_z = -0.5;
  double _max_x = 0.5;
  double _max_y = 0.5;
  double _max_z = 0.5;

  cv::Mat _image2;
  std::vector<cv::Point2d> _path2;
  std::vector<double> _heights2;
  double _scale2 = 1.0;
  double _min_x2 = -0.5;
  double _min_y2 = -0.5;
  double _min_z2 = -0.5;
  double _max_x2 = 0.5;
  double _max_y2 = 0.5;
  double _max_z2 = 0.5;
  
  const double _frameScale = 0.2;  // [m]
  std::atomic_bool drawing_;
  std::atomic_bool showing_;
};
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
    //if (!MyGUI::Manager::init())
    //{
    //   fprintf(stdout, "Failed to initialize GLFW\n");
    //   return -1;
    // }

    ImuParams params;
    params.rate = 200;
    params.gyro_div =  16.4 *180.0/M_PI;
    params.accel_div = 8192.0/9.81;
    SerialSyncImu imu0("/dev/cu.usbmodem1412101", &params);
    std::this_thread::sleep_for( std::chrono::duration<double, std::milli>(10));
    printf("Camera initialization started...\n");
    D435Camera camera;
    imu0.start();
    camera.enableSync(true);

    printf("Camera initialization complete\n");
    fflush(stdout);

    //Window for displaying the path
    PoseViewer poseViewer;
    /*MyGUI::CameraWindow path_win("Path Viewer", 1024, 620);
    MyGUI::ImageWindow img_win("Frame Viewer", 640,480, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    MyGUI::Path path1("path1", Eigen::Vector3d(1, 0, 0));
    MyGUI::Axis axis1("axis1", 1);
    MyGUI::Axis axis2("axis2", 1);
    MyGUI::Grid grid1("grid1", 10, 1);
    path_win.add_object(&path1);
    path_win.add_object(&axis1);
    path_win.add_object(&axis2);
    path_win.add_object(&grid1);*/
    //path_win.add_object(&(poseViewer._axis));
    //path_win.add_object(&(poseViewer._path3d));
    slam.okvis_estimator_->setFullStateCallback(
      std::bind(&PoseViewer::publishFullStateAsCallback, &poseViewer,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4));

    //Recieves output from SLAM system and displays to the screen
    /*FrameAvailableHandler handler([&path1, &axis2, &img_win](MultiCameraFrame frame) {
        Eigen::Matrix4d eigen_Tcw;
        cv::cv2eigen(frame.vecTcw[0],eigen_Tcw);
        Eigen::Affine3d transform(eigen_Tcw.inverse());
        path1.add_node(transform.translation());
        axis2.set_transform(transform);
        //img_win.set_image(frame.images[0]);
    });
    slam.AddFrameAvailableHandler(handler, "mapping");
*/
    //run until display is closed
    okvis::Time start(0.0);
    int id =0;
    while (true){ //MyGUI::Manager::running()){
        printf("test\n");
        //Update the display
        poseViewer.display();

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

        cv::imshow("Img 0", frame.images[0]);
        cv::imshow("Img 1", frame.images[1]);

        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27) break; // 27 is ESC
    }
    printf("\nTerminate...\n");
    // Clean up
    //slam.ShutDown();
    printf("\nExiting...\n");
    return 0;
}
