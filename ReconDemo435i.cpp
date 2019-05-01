#include "D435iCamera.h"
#include "serial_sync_imu.h"
#include "OkvisSLAMSystem.h"
#include <SaveFrame.h>
#include <iostream>
#include <thread>
#include "glfwManager.h"

#include <GL/glut.h>
#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <sys/stat.h>

using namespace ark;
using namespace cv;
using namespace std;

//OpenGL global variable
float window_width = 800;
float window_height = 800;
float xRot = 15.0f;
float yRot = 0.0f;
float xTrans = 0.0;
float yTrans = 0;
float zTrans = -35.0;
int ox;
int oy;
int buttonState;
float xRotLength = 0.0f;
float yRotLength = 0.0f;
bool wireframe = false;
bool stop = false;
thread *app;

void display_func() {
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glPushMatrix();

    if (buttonState == 1) {
        xRot += (xRotLength - xRot) * 0.1f;
        yRot += (yRotLength - yRot) * 0.1f;
    }

    glTranslatef(xTrans, yTrans, zTrans);
    glRotatef(xRot, 1.0f, 0.0f, 0.0f);
    glRotatef(yRot, 0.0f, 1.0f, 0.0f);

    // pointCloudGenerator->Render();

    glPopMatrix();
    glutSwapBuffers();

}

void idle_func() {
    glutPostRedisplay();
}

void reshape_func(GLint width, GLint height) {
    window_width = width;
    window_height = height;

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0, (float) width / height, 0.001, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -3.0f);
}

void application_thread() {
    // slam->Start();
    // pointCloudGenerator->Start();
    // bridgeRSD435->Start();

    // Main loop
    int tframe = 1;

    // while (!slam->IsRunning()) {
    //     cv::Mat imBGR, imD;
    //     cv::Mat imRGB;

    //     bridgeRSD435->GrabRGBDPair(imBGR, imD);

    //     cv::cvtColor(imBGR, imRGB, cv::COLOR_BGR2RGB);

    //     imBGR.release();

    //     // Pass the image to the SLAM system
    //     slam->PushFrame(imRGB, imD, tframe);
    // }
}

void keyboard_func(unsigned char key, int x, int y) {
    if (key == ' ') {
        if (!stop) {
            app = new thread(application_thread);
            app = new thread(application_thread);
            stop = !stop;
        } else {
            // slam->RequestStop();
            // updateKeyFrames();
            // pointCloudGenerator->RequestStop();
            // bridgeRSD435->Stop();
        }
    }

    if (key == 'w') {
        zTrans += 0.3f;
    }

    if (key == 's') {
        zTrans -= 0.3f;
    }

    if (key == 'a') {
        xTrans += 0.3f;
    }

    if (key == 'd') {
        xTrans -= 0.3f;
    }

    if (key == 'q') {
        yTrans -= 0.3f;
    }

    if (key == 'e') {
        yTrans += 0.3f;
    }

    if (key == 'p') {
        // slam->RequestStop();
        // pointCloudGenerator->RequestStop();
        // bridgeRSD435->Stop();
        // updateKeyFrames();
        // pointCloudGenerator->SavePly();
    }

    if (key == 'v')
        wireframe = !wireframe;


    glutPostRedisplay();
}

void mouse_func(int button, int state, int x, int y) {
    if (state == GLUT_DOWN) {
        buttonState = 1;
    } else if (state == GLUT_UP) {
        buttonState = 0;
    }

    ox = x;
    oy = y;

    glutPostRedisplay();
}

void motion_func(int x, int y) {
    float dx, dy;
    dx = (float) (x - ox);
    dy = (float) (y - oy);

    if (buttonState == 1) {
        xRotLength += dy / 5.0f;
        yRotLength += dx / 5.0f;
    }

    ox = x;
    oy = y;

    glutPostRedisplay();
}

int main(int argc, char **argv)
{

    if (argc >4 ) {
        std::cerr << "Usage: ./" << argv[0] << " configuration-yaml-file [vocabulary-file] [skip-first-seconds]" << std::endl
        <<"Args given: " << argc << std::endl;
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
    else configFilename = "intr.yaml";

    std::string vocabFilename;
    if (argc > 2) vocabFilename = argv[2];
    else vocabFilename = "brisk_k10_l6_t40_n75_65k_scale_2oct.yml.bn";

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
    //SerialSyncImu imu0("COM4", &params);
    printf("Camera initialization started...\n");
    fflush(stdout);
    D435iCamera camera;
    printf("IMU initialization started...\n");
    //std::this_thread::sleep_for( std::chrono::duration<double, std::milli>(500));
    //while(!imu0.start());
    //std::this_thread::sleep_for( std::chrono::duration<double, std::milli>(500));
    camera.start();

    printf("Camera-IMU initialization complete\n");
    fflush(stdout);

    //Window for displaying the path
    //PoseViewer poseViewer;
    MyGUI::ReconWindow recon_win("Reconstruction Viewer", 800, 800);
    MyGUI::CameraWindow traj_win("Traj Viewer", 600,600);
    MyGUI::ARCameraWindow ar_win("AR Viewer", 640*2.5,480*2.5, GL_LUMINANCE, GL_UNSIGNED_BYTE, 3.84299896e+02, 3.84299896e+02, 3.22548157e+02, 2.36944305e+02,0.01,100);
    traj_win.set_pos(640*2.5,100);
    ar_win.set_pos(0,100);
    MyGUI::Path path1("path1", Eigen::Vector3d(1, 0, 0));
    MyGUI::Axis axis1("axis1", .1);
    MyGUI::Axis axis2("axis2", 1);
    MyGUI::Grid grid1("grid1", 10, 1);
    traj_win.add_object(&path1);
    traj_win.add_object(&axis1);
    traj_win.add_object(&axis2);
    traj_win.add_object(&grid1);
    recon_win.add_object(&grid1);
    ar_win.add_object(&axis1);

    //Create saveFrame. It stores key frames into timestamp, RGB image, depth image folders
    ark::SaveFrame* saveFrame = new ark::SaveFrame("./frames/");

    int count = 0;

    KeyFrameAvailableHandler key_handler([&count, &saveFrame](MultiCameraFrame::Ptr frame){
        //create an empty rgbd frame
        //convert frame to rgbd frame
        //call save frame
        //increment count
        std::cout << "key frame: " << count << std::endl;
        RGBDFrame rgbdframe;
        rgbdframe.frameId = count;
        frame->getRGBDFrame(rgbdframe);


        saveFrame->frameWrite(rgbdframe);


            // string folder = "../frames/";

            // std::cout << "save keyframe id: " << frame->frameId_ << std::endl;

            // // // get color
            // if (!frame->images_[3].empty()){
            //     std::cout << "got color" << std::endl;
            //     cv::imwrite(folder + "RGB/" + std::to_string(count) + ".png", frame->images_[3]);
            // }

            // // get depth
            // if (!frame->images_[2].empty()){
            // std::cout << "Depth " << frame->frameId_ << std::endl;
            // cv::Mat depth255;

            
            // frame->images_[2].convertTo(depth255, CV_16UC1, 1000);
            // cv::imwrite(folder + "depth/" + std::to_string(count) + ".png", depth255);
            // }

            // cv::Mat twc(4,4,CV_32FC1);
            // // Eigen::Matrix4d T_WS_m = frame->T_WS()
            // for(int ii = 0; ii < 4; ii ++){
            //     for(int jj = 0;jj < 4; jj ++){
            //         twc.at<float>(ii, jj) = frame->T_WS()(ii,jj);
            //     }
            // }

            // cv::FileStorage fs(folder + "tcw/" + std::to_string(count)+".txt",cv::FileStorage::WRITE);
            // fs << "twc" << twc.inv();

            // fs.release();

            // std::cout << "finished writing " << frame->frameId_ << " to " << count << std::endl;

            count ++;
    });

    slam.AddKeyFrameAvailableHandler(key_handler, "saving");

    //Recieves output from SLAM system and displays to the screen
    FrameAvailableHandler handler([&path1, &axis2, &ar_win](MultiCameraFrame::Ptr frame) {
        Eigen::Affine3d transform(frame->T_WS());
        path1.add_node(transform.translation());
        axis2.set_transform(transform);
        ar_win.set_camera(transform);
        ar_win.set_image(frame->images_[0]);

    });
    slam.AddFrameAvailableHandler(handler, "mapping");

    LoopClosureDetectedHandler loopHandler([&slam, &path1](void) {
        std::vector<Eigen::Matrix4d> traj;
        slam.getTrajectory(traj);
        path1.clear();
        for(size_t i=0; i<traj.size(); i++){
            path1.add_node(traj[i].block<3,1>(0,3));
        }
    });
    slam.AddLoopClosureDetectedHandler(loopHandler,"trajectoryUpdate");
    //run until display is closed
    okvis::Time start(0.0);
    int id =0;

    //glut window

    // glutInit(&argc, argv);
    // glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    // glutInitWindowSize(window_width, window_height);
    // (void) glutCreateWindow("GLUT Program");

    // init();

    // glutSetWindowTitle("OpenARK 3D Reconstruction");
    // glutDisplayFunc(display_func);
    // glutReshapeFunc(reshape_func);
    // glutIdleFunc(idle_func);
    // glutMouseFunc(mouse_func);
    // glutMotionFunc(motion_func);
    // glutKeyboardFunc(keyboard_func);
    // glutMainLoop();

    while (MyGUI::Manager::running()){
        //printf("test\n");
        //Update the display
        MyGUI::Manager::update();

        //Get current camera frame
        MultiCameraFrame frame;
        camera.update(frame);

        //Get timestamp of camera
        //printf("getTs: %i\n",frame.frameId_);
        //frame.timestamp_ = imu0.getFrameTimestamp(frame.frameId_);//
        //printf("frame ts: %f\n",frame.timestamp_/1e9);
        //if(frame.timestamp_<0)
        //    continue;

        //Get or wait for IMU Data until current frame 
        //std::cout << "frame: " << frame.timestamp_ << std::endl;
        std::vector<ImuPair> imuData;
        camera.getImuToTime(frame.timestamp_,imuData);
        //std::cout << "numimu: " << imuData.size() << std::endl;

        //Add data to SLAM system
        slam.PushIMU(imuData);
        slam.PushFrame(frame.images_, frame.timestamp_);

        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27) break; // 27 is ESC
        //if (k == 'p')

    }
    printf("\nTerminate...\n");
    // Clean up
    slam.ShutDown();
    printf("\nExiting...\n");
    return 0;
}
