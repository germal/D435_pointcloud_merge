// Copyright (c) 2018-20, Polaris3D. All rights reserved.
#pragma once

#include <memory>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
// Cause: librealsense2/rs.hpp should not be included here to avoid unncessary dependencies
#include <vector>
#if defined(__build_ans_wrapper__)
#include <vsys.h>
#endif

class RsD4xx {
public:
  enum {
    MODE_480x270_90HZ = 0,
    MODE_480x270_60HZ,
    MODE_480x270_30HZ, 
    MODE_480x270_15HZ,
    MODE_480x270_06HZ,
    MODE_640x480_30HZ
  };
  RsD4xx(const int mode = MODE_480x270_60HZ);
  ~RsD4xx();
  bool init(int i);
  bool initialized() { return bInit_; }
  /** @brief  Proceed one scan. */
  bool proceed();
  const size_t size() { return vs_; }
  std::vector<float> get_pv_(){ return pv_;}
  /** @brief  Return pointer to the i-th point containing (x,y,z). 
   * It is only valid if the size of cloud is larger than zero. */
  const float* operator[](size_t i);
  const int get_height() {return hei_;}
  const int get_width() {return wid_;}
  const std::string get_serial() {return serial_number;}  
#if defined(__build_ans_wrapper__)
  void copyToSync(v::PointCloud::Ptr pc);
  void copyToUnSync(v::PointCloud::Ptr pc);
#endif

private:
  std::shared_ptr<void> ppipe_;
  std::shared_ptr<void> pd2p_;
  std::vector<float> pv_;
  size_t vs_;
  bool bInit_;
  const int wid_,hei_,fps_;
  rs2::context* cxt;
  rs2::device* dev;
  rs2::device_list* devices;
  std::string serial_number;
};