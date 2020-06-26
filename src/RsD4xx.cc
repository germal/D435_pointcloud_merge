#include "RsD4xx.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <glog/logging.h>

DEFINE_int32(rs_wid_stride, 2, "Stride of depth width (uniform subsampling)");
DEFINE_int32(rs_hei_stride, 10, "Stride of depth height (uniform subsampling)");

static int MODES_DEPTH[][3] = { 
  {480, 270, 90}, {480, 270, 60}, {480, 270, 30}, {480, 270, 15}, {480, 270, 6}, 
  {640, 480, 30} };

RsD4xx::RsD4xx(const int mode) 
  : ppipe_(new rs2::pipeline)
  , pd2p_(new rs2::pointcloud)
  , bInit_(false)
  , cxt(new rs2::context)
  , dev(new rs2::device)
  , devices(new rs2::device_list)
  , vs_(0)
  , wid_(MODES_DEPTH[mode][0]), hei_(MODES_DEPTH[mode][1]), fps_(MODES_DEPTH[mode][2])

{}

RsD4xx::~RsD4xx() {
  //LOG(INFO) << "Destroy RsD4xx";
}

bool RsD4xx::init(int i) {
  rs2_error* e = 0;
	*devices = cxt->query_devices();
  *dev = (*devices)[i];
  LOG(INFO) << "Hardware resetting device " << i + 1 << ", Serial number: " << dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  dev->hardware_reset();
  serial_number = std::string(dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  if (devices->size() == 0) return false;
  LOG(INFO)<<"Number of devices: " << devices->size();
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  std::shared_ptr<rs2::pointcloud> pd2p = std::static_pointer_cast<rs2::pointcloud>(pd2p_);
  // Contruct a pipeline which abstracts the device
  // Create a configuration for configuring the pipeline with a non default profile
  std::shared_ptr<rs2::pipeline> pp = std::static_pointer_cast<rs2::pipeline>(ppipe_);
  rs2::config cfg;
  LOG(INFO) << "Configure depth sensor (" << wid_ << "x" << hei_ << "@" << fps_ <<")";
  //cfg.enable_stream(RS2_STREAM_INFRARED, wid_, hei_, RS2_FORMAT_Y8, fps_);
  cfg.enable_stream(RS2_STREAM_DEPTH, wid_, hei_, RS2_FORMAT_Z16, fps_);
  cfg.enable_device(dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  //Instruct pipeline to start streaming with the requested configuration
  pp->start(cfg);
  bInit_ = true;
  pv_.resize(wid_*hei_*3);
  // profile.get_device().query_sensors().size();
  LOG(INFO) << "Finished Initializing camera " << i + 1;
  return bInit_;
}

template<class MAP_DEPTH> 
size_t deproject_depth(float * points, const rs2_intrinsics & intrin, const uint16_t * depth, 
  MAP_DEPTH map_depth, const int ws, const int hs)
{
  size_t n = 0;
  for (int y = 0; y < intrin.height; y+=hs)
  {
    const uint16_t *pd = depth + y*intrin.width;
    for (int x = 0; x < intrin.width; x+=ws)
    {
      const float pixel[] = { (float)x, (float)y };
      rs2_deproject_pixel_to_point(points, &intrin, pixel, map_depth(pd[x]));
      points+=3;
      n++;
    }
  }
  return n;
}

static
size_t depth_to_points(float* output, 
    const rs2_intrinsics &depth_intrinsics, const rs2::depth_frame& depth_frame, const float depth_scale, const int wstride = 1, const int hstride = 1)
{
  return deproject_depth(output, depth_intrinsics, (const uint16_t*)depth_frame.get_data(), 
    [depth_scale](uint16_t z) { return depth_scale * z; }, wstride, hstride);
}

bool RsD4xx::proceed() {
  LOG(INFO) <<"Initiate Proceed: " << serial_number;
  //if (!bInit_) init();
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  std::shared_ptr<rs2::pointcloud> pd2p = std::static_pointer_cast<rs2::pointcloud>(pd2p_);
  // Contruct a pipeline which abstracts the device
  // Create a configuration for configuring the pipeline with a non default profile
  std::shared_ptr<rs2::pipeline> pp = std::static_pointer_cast<rs2::pipeline>(ppipe_);
  auto profile = pp->get_active_profile();
  rs2_intrinsics intrin = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
  auto sensors = profile.get_device().query_sensors();
  const float depth_unit = sensors[0].get_option(RS2_OPTION_DEPTH_UNITS);
  rs2::frameset frames; 
  if (pp->poll_for_frames(&frames)) {
    // Get depth/color frames.
    rs2::frame depth = frames.get_depth_frame();
    // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
    // auto color = frames.get_color_frame();
    // if (!color) color = frames.get_infrared_frame();
    // Generate the pointcloud and texture mappings
#if 0
    rs2::points points = pd2p->calculate(depth); vs_ = points.size();
    memcpy(&pv_[0], points.get_vertices(), points.size()*3*sizeof(float));
#else
    size_t np = depth_to_points(&pv_[0], intrin, depth, depth_unit, FLAGS_rs_wid_stride, FLAGS_rs_hei_stride); vs_ = np;
#endif
    // d2p.map_to(color);
    // const rs2::vertex *v = points.get_vertices();
    // const rs2::texture_coordinate *tc = points.get_texture_coordinates();
    return true;
  }
  return false;
}

const float* RsD4xx::operator[](size_t i) { return &pv_[i*3]; }

#if defined(__build_ans_wrapper__)
void RsD4xx::copyToSync(v::PointCloud::Ptr pc) {
  if (!pc) pc.reset(new v::PointCloud);
  pc->d.clear();
  for (size_t i = 0; i < size(); i++) {
    const float *pvi = &pv_[i*3];
    pc->d.emplace_back(v::PointXYZRGBALT(pvi[0], pvi[1], pvi[2], 0, 0, 0));
  }
}
void RsD4xx::copyToUnSync(v::PointCloud::Ptr pc) {
  if (!pc) pc.reset(new v::PointCloud);
  pc->u.clear();
  for (size_t i = 0; i < size(); i++) {
    const float *pvi = &pv_[i*3];
    pc->u.emplace_back(v::PointXYZRGBALT(pvi[0], pvi[1], pvi[2], 0, 0, 0));
  }
}
#endif