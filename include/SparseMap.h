#ifndef _SPARSE_MAP_H_
#define _SPARSE_MAP_H_


#include "Types.h"
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/eigen.hpp>
#include <DBoW2.h>
#include <DLoopDetector.h>
#include "PoseGraphSolver.h"
#include "CorrespondenceRansac.h"
#include "Util.h"
#include "PointCostSolver.h"

namespace ark{

/**
 * @brief This class 
 */
template<class TDescriptor, class F>
class SparseMap {
 public:

  SparseMap(bool useLoopClosures = false, std::string vocabPath = "", bool binaryVocab = true, cv::DescriptorMatcher* matcher = nullptr):
  useLoopClosures(useLoopClosures),matcher_(matcher),bowId(0),currentKeyframeId(-1)
  {
      if(useLoopClosures){
        std::cout << "Loading Vocabulary From: " << vocabPath << std::endl;
        vocab_.reset(new DBoW2::TemplatedVocabulary<TDescriptor, F>());
        if(!binaryVocab){
          vocab_->load(vocabPath);
        }else{
          vocab_->binaryLoad(vocabPath); //Note: Binary Loading only supported for ORB/BRISK vocabularies
        }
        vocab_->save(vocabPath+std::string(".tst"));
        std::cout << "Vocabulary Size: " << vocab_->size() << std::endl;
        //db_.reset(new DBoW2::TemplatedDatabase<TDescriptor, F>());
        //db_->setVocabulary(*vocab_);
        typename DLoopDetector::TemplatedLoopDetector<TDescriptor, F>::Parameters detectorParams(0,0);
        //can change specific parameters if we want
        // Parameters given by default are:
        // use nss = true
        // alpha = 0.3
        // k = 3
        // geom checking = GEOM_DI
        detectorParams.di_levels = 4;
        detector_.reset(new DLoopDetector::TemplatedLoopDetector<TDescriptor, F>(*vocab_,detectorParams));
        std::cout << "Map Initialized\n";
        //detector_->setDatabase(*db_);


      }
  }

  void getTrajectory(std::vector<Eigen::Matrix4d>& trajOut){
    trajOut.resize(frameMap_.size());
    size_t i=0;
    for(std::map<int, MultiCameraFrame::Ptr>::iterator frame = frameMap_.begin(); 
        frame!=frameMap_.end(); frame++, i++){
      trajOut[i] = frame->second->T_WS();
    }
  }

  MultiCameraFrame::Ptr getCurrentKeyframe(){
    return getKeyframe(currentKeyframeId);
  };
  MultiCameraFrame::Ptr getKeyframe(int frameId){
    std::map<int,MultiCameraFrame::Ptr>::iterator it = frameMap_.find(frameId);
    if(it!=frameMap_.end()){
      return it->second;
    }else {
      return MultiCameraFrame::Ptr(nullptr);
    }
  }
  
  bool addKeyframe(MultiCameraFrame::Ptr kf){
    //std::cout << "PROCESS KEYFRAME: " << kf->frameId_ << std::endl;
    frameMap_[kf->frameId_]=kf;
    kf->previousKeyframeId_ = currentKeyframeId;
    kf->previousKeyframe_ = getCurrentKeyframe();

    Eigen::Matrix4d T_K1K2; 
    if(kf->previousKeyframe_.get()!=nullptr){
      T_K1K2 = kf->previousKeyframe_->T_WS_.inverse()*kf->T_WS_;
      kf->setOptimizedTransform(kf->previousKeyframe_->T_WS()*T_K1K2);
    }else 
      T_K1K2 = Eigen::Matrix4d::Identity();


    currentKeyframeId = kf->frameId_;
    graph_.AddPose(kf->frameId_,kf->T_WS());
    graph_.AddConstraint(kf->previousKeyframeId_,kf->frameId_,T_K1K2);

    //std::cout << kf->keypoints(0).size() << " " << kf->descriptors(0).size() << std::endl << std::flush;
    if(useLoopClosures){
      bowFrameMap_[bowId]=kf;
      //convert to descriptors to DBoW descriptor format
      std::vector<cv::Mat> bowDesc;
      kf->descriptorsAsVec(0,bowDesc);

      DLoopDetector::DetectionResult result;
      DBoW2::BowVector bowVec;
      DBoW2::FeatureVector featvec;
      detector_->detectLoop(kf->keypoints(0),bowDesc,result);
      bowId++;
      MultiCameraFrame::Ptr loop_kf;
      if(result.detection())
      {
        std::cout << result.match << " " << bowId << std::endl; 
        loop_kf = bowFrameMap_[result.match];
        cout << "- Loop found with image " << loop_kf->frameId_ << "!"
          << endl;
      }else{
        return false; //pose added to graph, no loop detected, nothing left to do
      }

      //transform estimation
      //should move to function to be set as one of a variety of methods

      //brute force matching
      std::vector<cv::DMatch> matches; 
      //query,train
      matcher_->match(kf->descriptors(0),loop_kf->descriptors(0), matches);
      std::cout << "sizes: kf: " << kf->descriptors(0).rows << " loop_kf: "
        << loop_kf->descriptors(0).rows << " matches: " << matches.size() << std::endl;

      //depth map time
      //get full pointcloud for each frame
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr kf_cloud = 
        util::toPointCloud<pcl::PointXYZ>(kf->images_[2]);
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr loop_kf_cloud = 
        util::toPointCloud<pcl::PointXYZ>(loop_kf->images_[2]);

      //get feature point clouds
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr kf_feat_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for(int i=0; i<kf->keypoints(0).size(); i++){
        kf_feat_cloud->points.push_back((*kf_cloud)(kf->keypoints(0)[i].pt.x,kf->keypoints(0)[i].pt.y));
      } 
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr loop_kf_feat_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for(int i=0; i<loop_kf->keypoints(0).size(); i++){
        loop_kf_feat_cloud->points.push_back((*loop_kf_cloud)(loop_kf->keypoints(0)[i].pt.x,loop_kf->keypoints(0)[i].pt.y));
      }           

      //convert DMatch to correspondence
      std::vector<int> correspondences(matches.size());
      for(int i=0; i<matches.size(); i++){
        correspondences[matches[i].queryIdx]=matches[i].trainIdx;
      }

      int numInliers;
      std::vector<bool> inliers;
      Eigen::Affine3d transformEstimate;
      //find initial transform estimate using feature points
      CorrespondenceRansac<pcl::PointXYZ>::getInliersWithTransform(
            kf_feat_cloud, loop_kf_feat_cloud, correspondences,
            3, 0.2, 50, numInliers, inliers, transformEstimate);
      if(((float)numInliers)/correspondences.size()<0.1)
        return false; //transform unreliable
      std::cout << "inliers: " << numInliers << " / " << correspondences.size() << std::endl;

      transformEstimate = PointCostSolver<pcl::PointXYZ>::solve(kf_feat_cloud,loop_kf_feat_cloud,
                                            correspondences, inliers, transformEstimate);

      std::cout << transformEstimate.matrix() << std::endl;
      //refine transform using full depth map
      //TODO: this is probably a bad idea, need to store depth map of all keyframes...
      //though could save to file.. which may be less of a bad idea

      //transformEstimate = ICP<pcl::PointXYZ>::match(kf_cloud,loop_kf_cloud,3,0.2,50,3,transformEstimate);
      graph_.AddConstraint(kf->frameId_,loop_kf->frameId_,transformEstimate.matrix());
      graph_.optimize();
      for(std::map<int, MultiCameraFrame::Ptr>::iterator frame=frameMap_.begin();
        frame!=frameMap_.end(); frame++){
        frame->second->setOptimizedTransform(graph_.getTransformById(frame->first));
      }

      return true;

      //OR refine transform using reprojection error?
      //should test both
      //reprojection error may be better

      //add constraint to pose graph and optimize



    }
    return false;

/*
      //get the 3d position of each matched point
      //and create new feature vector that only uses the points that were matched
      std::vector<cv::Point3f> matchedPoints3d;
      for (size_t k = 0; k < matchedFrame->observations.size(); ++k) {
        if(valid[k]){
          Eigen::Vector4d matchedPoint4d = matchedFrame->observations[k].landmark_C;
          cv::Point3f matchedPoint3d(matchedPoint4d[0],matchedPoint4d[1],matchedPoint4d[2]);
          matchedPoints3d.push_back(matchedPoint3d);
          newFeaturesFinal.push_back(newFeatures[k]);
        }
      }

      //get intrinsics and distortion coeffs from fame
      Eigen::VectorXd full_intrinsics;
      kf->keyFrames->geometry(0)->getIntrinsics(full_intrinsics);

      cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
      cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
      cameraMatrix.at<double>(0,0) = full_intrinsics[0];
      cameraMatrix.at<double>(1,1) = full_intrinsics[1];
      cameraMatrix.at<double>(0,2) = full_intrinsics[2];
      cameraMatrix.at<double>(1,2) = full_intrinsics[3];

      distCoeffs.at<double>(0) = full_intrinsics[4];
      distCoeffs.at<double>(1) = full_intrinsics[5];
      distCoeffs.at<double>(2) = full_intrinsics[6];
      distCoeffs.at<double>(3) = full_intrinsics[7];

      //compute the 3d position of the new keyframe relative to the matched frame
      //we will use ransac to determine which points are inliers
      cv::Mat rvec(3,1,cv::DataType<double>::type);
      cv::Mat tvec(3,1,cv::DataType<double>::type);

      std::vector<int> pnpInliers; 

      bool pnp_success = false;
      if(newFeaturesFinal.size()>30){
        pnp_success = solvePnPRansac(matchedPoints3d, newFeaturesFinal, 
          cameraMatrix, distCoeffs, rvec, tvec, false, 100, 2.0,
          0.5,pnpInliers);
      }

      //convert to eigen transform
      cv::Mat R; 
      cv::Rodrigues(rvec,R);
      tvec= -R*tvec;

      Eigen::Matrix3d rot;
      cv::cv2eigen(R,rot);

      Eigen::Vector3d trans;
      cv::cv2eigen(tvec,trans);
      Eigen::Affine3d transform_estimate = Eigen::Translation3d(trans)*Eigen::Quaterniond(rot);

      //check if transform is valid, meaning we were able to compute
      //a transform that enough points agreed upon
      if(pnp_success && pnpInliers.size()/(float)newFeaturesFinal.size()>0.301){
        //.301 was experimentally determined to give good results
        //it is the % required inliers for the loop closure to be good

        loopClosure=true;

      }
    }
    lastEntry_ = kf->id = db_->add(kf->bowVec);*/

  }


  std::unique_ptr<DBoW2::TemplatedVocabulary<TDescriptor, F> >vocab_;
  //std::unique_ptr<DBoW2::TemplatedDatabase<TDescriptor, F> > db_;
  std::unique_ptr<DLoopDetector::TemplatedLoopDetector<TDescriptor, F> >detector_;
  std::shared_ptr<cv::DescriptorMatcher> matcher_;
  std::map<int, MultiCameraFrame::Ptr> frameMap_;
  std::map<int, MultiCameraFrame::Ptr> bowFrameMap_;

  DBoW2::EntryId lastEntry_;
  bool useLoopClosures;
  int currentKeyframeId;
  int bowId;

  SimplePoseGraphSolver graph_;



private: 

 };//class SparseMap


} //namespace ark


#endif //_SPARSE_MAP_H_
