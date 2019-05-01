//
// Created by yiwen on 2/2/19.
//
/*
Esther commented
- Enables online frame writing to folder for offline reconstruction later
- Enables offline file access
- Requires onKeyFrameAvailable handler in SLAM
- Used in rgbd_realsense_d435, rgbd_realsense_load_gl, rgbd_realsense_load_categorized

*/

#include <chrono>
#include <mutex>
//#include <Types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>

//#include <MathUtils.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/fast_bilateral.h>
// #include <opencv2/ximgproc.hpp>
#include <opencv2/opencv.hpp>
#include "SaveFrame.h"

namespace ark {

    void createFolder(struct stat &info, std::string folderPath){
        if(stat( folderPath.c_str(), &info ) != 0 ) {
            if (-1 == mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
            {
                std::cout<< "Error creating directory "<< folderPath<<" !" << std::endl;
                exit(1);
            }
            std::cout << folderPath << " is created" << folderPath << std::endl;
        }else if( info.st_mode & S_IFDIR )  // S_ISDIR() doesn't exist on my windows
            std::cout<<folderPath<<" is a directory"<<std::endl;
        else
            std::cout<<folderPath<<" is no directory"<<std::endl;
    }


    SaveFrame::SaveFrame(std::string folderPath) {

        struct stat info;

        createFolder(info, folderPath);

        rgbPath = folderPath +"RGB/";
        depthPath = folderPath +"depth/";
        tcwPath = folderPath +"tcw/";

        createFolder(info, rgbPath);
        createFolder(info, depthPath);
        createFolder(info, tcwPath);
        createFolder(info, folderPath + "tcw_loop/");

        mKeyFrame.frameId = -1;
        mbRequestStop = false;
    }


    void SaveFrame::OnKeyFrameAvailable(const RGBDFrame &keyFrame) {
        if (mMapRGBDFrame.find(keyFrame.frameId) != mMapRGBDFrame.end())
            return;
        std::cout << "OnKeyFrameAvailable" << keyFrame.frameId << std::endl;
        // std::unique_lock<std::mutex> lock(mKeyFrameMutex);
        keyFrame.mTcw.copyTo(mKeyFrame.mTcw);
        keyFrame.imRGB.copyTo(mKeyFrame.imRGB);
        keyFrame.imDepth.copyTo(mKeyFrame.imDepth);

        mKeyFrame.frameId = keyFrame.frameId;
        mMapRGBDFrame[keyFrame.frameId] = ark::RGBDFrame();
    }

    void SaveFrame::OnFrameAvailable(const RGBDFrame &frame) {
        std::cout << "OnFrameAvailable" << frame.frameId << std::endl;
    }

    void SaveFrame::frameWrite(const RGBDFrame &frame){
        if (mMapRGBDFrame.find(frame.frameId) != mMapRGBDFrame.end())
            return; 

        std::cout<<"frameWrite frame = "<< frame.frameId <<std::endl;

        cv::Mat imBGR;
        cv::cvtColor(frame.imRGB, imBGR, CV_RGB2BGR);
        cv::imwrite(rgbPath + std::to_string(frame.frameId) + ".png", imBGR);

        cv::Mat depth255;

        
        frame.imDepth.convertTo(depth255, CV_16UC1, 1000);
        cv::imwrite(depthPath + std::to_string(frame.frameId) + ".png", depth255);


        cv::FileStorage fs(tcwPath + std::to_string(frame.frameId)+".txt",cv::FileStorage::WRITE);
        fs << "tcw" << frame.mTcw;
        fs.release();

        std::cout << "finished writing " << frame.frameId << std::endl;



        //RGB and Depth to .xml (.png preferable)
        /*
        cv::FileStorage fs2(depth_to_tcw_Path + std::to_string(frame.frameId)+".xml",cv::FileStorage::WRITE);
        fs2 << "depth" << depth255;
        // fs << "rgb" << frame.imRGB;
        fs2.release();
        */

        mMapRGBDFrame[frame.frameId] = ark::RGBDFrame();

    }

    RGBDFrame SaveFrame::frameLoad(int frameId){
        std::cout<<"frameLoad start = "<<frameId<<std::endl;

        RGBDFrame frame;

        frame.frameId = frameId;


        frame.imRGB = cv::imread(rgbPath + std::to_string(frame.frameId) + ".png",cv::IMREAD_COLOR);

        if(frame.imRGB.rows == 0){
                    std::cout<<"frameLoad RGB fail = "<<frameId<<std::endl;
            frame.frameId = -1;
            return frame;
        }

        //cv::resize(rgbBig, frame.imRGB, cv::Size(640,480));

        //rgbBig.release();
 
        cv::Mat depth255 = cv::imread(depthPath + std::to_string(frame.frameId) + ".png",-1);

        depth255.convertTo(frame.imDepth, CV_32FC1);

        depth255.release();

        frame.imDepth *= 0.001;
        


        //TCW FROM XML
        
        cv::FileStorage fs2(tcwPath + std::to_string(frame.frameId) + ".txt", cv::FileStorage::READ);
        fs2["twc"] >> frame.mTcw;
        fs2.release();


        if(frame.mTcw.rows == 0) {
                                std::cout<<"frameLoad tcw fail = "<<frameId<<std::endl;

            frame.frameId = -1;
            return frame;
        }

        
        std::cout<<"frameLoad frame = "<<frameId<<std::endl;

        /*
        //TCW FROM TEXT
        float tcwArr[4][4];
        std::ifstream tcwFile;
        tcwFile.open(tcwPath + std::to_string(frame.frameId) + ".txt");
        for (int i = 0; i < 4; ++i) {
            for (int k = 0; k < 4; ++k) {
                tcwFile >> tcwArr[i][k];
            }
        }
        cv::Mat tcw(4, 4, CV_32FC1, tcwArr);    
        frame.mTcw = tcw.inv();
        */

        return frame;
    }

}

