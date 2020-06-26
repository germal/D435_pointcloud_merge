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

  // Get YAML Configuration ///////////////////////////////////////
  // For now, assume there was a YAML configuration ///////////////
  std::map<string, Eigen::Transform<float, 3, Eigen::Affine>> serial_extrinsic;
  std::string standard = "935322070552";
  
  Eigen::Transform<float, 3, Eigen::Affine> t1 = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
  serial_extrinsic["935322070552"] = t1;  

  Eigen::Vector3f translation;
  translation << 0.1, 0.0, 0.0;
  Eigen::Matrix3f rotation;
  rotation << -1.0 , 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0; 
  Eigen::Transform<float, 3, Eigen::Affine> t2 = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
  t2.translate(translation);
  t2.rotate(rotation);
  
  serial_extrinsic["935422070991"] = t2;

  size_t numcameras = 2;
  int mode = 0;
  const int resolution = 129600;
  // End YAML Configuration /////////////////////////////////////////

  std::vector<RsD4xx*> cameras;
  for(int i = 0; i < numcameras; i++){
    RsD4xx* temp = new RsD4xx(mode);
    cameras.emplace_back(temp);
  }
  bool bInit = false;

  for(int i = numcameras - 1; i >=0 ; i--){
    LOG(INFO) << "Initializing Camera" << i + 1;
    bInit = cameras[i]->init(i);
    if (!bInit)
      {LOG(ERROR) << "Insufficient rs_cameras detected!";
      return -1;}
    cameras[i]->proceed();
  }
  LOG(INFO) << "All camera initialization complete";
  
  viz::Viz3d myWindow("Creating Widgets");

  int standard_idx;
  for(int i = 0; i < numcameras; i++){
    if(standard == cameras[i]->get_serial()){
      standard_idx = i;
      break;
    }
  }

  while(1){
    Eigen::MatrixXf std;
    Eigen::MatrixXf temp;

    //Get frame from std camera first
    while(!cameras[standard_idx]->proceed()){
      LOG(ERROR) << "Waiting for standard camera to proceed successfully";
    }
    auto pv = cameras[standard_idx]->get_pv_();
    std = Eigen::Map<Eigen::Matrix<float, 3, resolution >>(pv.data());

    //get frames from other cameras
    for(int i = 0; i < numcameras; i ++){
      if (i == standard_idx) continue;
      while(!cameras[i]->proceed()){
        LOG(ERROR) << "Waiting for camera to proceed successfully: camera " << i + 1;
      }
      auto pv = cameras[i]->get_pv_();
      temp = Eigen::Map<Eigen::Matrix<float, 3, resolution >>(pv.data());

      //Merge the matrices together
      Eigen::Transform<float, 3, Eigen::Affine> t = serial_extrinsic.find(cameras[i]->get_serial())->second;
      for(auto i=0; i < resolution; i++){
        temp.col(i) = t.linear()*temp.col(i) + t.translation();
      }
      // Combine the data from each camera
      Eigen::MatrixXf C(std.rows(), std.cols() + temp.cols());
      C << std, temp;
      std = C;
    }
    
    //Visualization
    std::vector<float> result(std.data(), std.data() + std.size());
    Mat *ir = new Mat(Size(cameras[0]->get_width() * numcameras, cameras[0]->get_height()), CV_32FC3, (void*)&result[0], Mat::AUTO_STEP);
    viz::WCloud pcloud(*ir, viz::Color::green());
    myWindow.showWidget("pointcloud", pcloud);
    myWindow.spinOnce(1, true);
   
    if (waitKey(5) == 27) break;
  }
  return 0;
}
