//#include <lasreader.hpp>
//#include <laswriter.hpp>
#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <Eigen/Eigen>

#include <iostream>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include "RsD4xx.h"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  size_t numcameras = 2;
  RsD4xx rs_0(RsD4xx::MODE_480x270_60HZ);
  RsD4xx rs_1(RsD4xx::MODE_480x270_60HZ);
  
  RsD4xx cameras[numcameras] = {rs_0, rs_1};
  //Initialize all cameras
  //numcameras = 1;
  bool bInit = false;

  for(int i = numcameras - 1; i >= 0; i--){
    LOG(INFO) << "Initializing Camera " << i + 1;
    bInit = cameras[i].init(i);
    if (!bInit) 
      {LOG(ERROR) << "Insufficient rs_cameras detected!"; 
      return -1;}
    cameras[i].proceed();
  }
  LOG(INFO) << "All camera intialization complete";

  viz::Viz3d myWindow("Creating Widgets");
  
  while(1){
    Mat* ir1;
    Mat* ir2;

    Eigen::MatrixXf mat1;
    Eigen::MatrixXf mat2;

    for(int i = 0; i < numcameras; i ++){
        while(!cameras[i].proceed()){
          LOG(ERROR) << "Waiting for camera to proceed successfully: camera " << i + 1;
        }
        if(i==0){
          auto pv = cameras[i].get_pv_();
          mat1 = Eigen::Map<Eigen::Matrix<float, 3, 129600 >>(pv.data());
        }
        else if(i==1){
          auto pv = cameras[i].get_pv_();
          mat2 = Eigen::Map<Eigen::Matrix<float, 3, 129600 >>(pv.data());
        }
    }
    // Define Rotation and Translation matrices
    Eigen::Vector3f translation;
    translation << 0.1, 0.0, 0.0;
    Eigen::Matrix3f rotation;
    rotation << -1.0 , 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0; 
    Eigen::Transform<float, 3, Eigen::Affine> t = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    t.translate(translation);
    t.rotate(rotation);
    
    // Apply the rotation and translation separately
    for(auto i=0; i < 129600; ++i){
        mat2.col(i) = t.linear()*mat2.col(i) + t.translation();
    }
    // Combine the data from each camera
    Eigen::MatrixXf C(mat1.rows(), mat1.cols() * numcameras);
    C << mat1, mat2;
    
    //Visualization
    std::vector<float> result(C.data(), C.data() + C.size());
    ir1 = new Mat(Size(cameras[0].get_width() * 2, cameras[0].get_height()), CV_32FC3, (void*)&result[0], Mat::AUTO_STEP);
    viz::WCloud pcloud(*ir1, viz::Color::green());
    myWindow.showWidget("pointcloud", pcloud);
    myWindow.spinOnce(1, true);
   
    if (waitKey(5) == 27) break;
  }
  return 0;
}
