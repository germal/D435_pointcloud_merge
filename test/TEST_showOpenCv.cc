// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

#include <glog/logging.h>
#include <gflags/gflags.h>


using namespace std;
using namespace cv;

static int CMODE_DEPTH = 0;
static int MODES_DEPTH[][3] = { {480, 270, 60}, {640, 480, 30} };
static int WID_DEPTH = MODES_DEPTH[CMODE_DEPTH][0];
static int HEI_DEPTH = MODES_DEPTH[CMODE_DEPTH][1];
static int FPS_DEPTH = MODES_DEPTH[CMODE_DEPTH][2];

int main(int argc, char *argv[])
{
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  //Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  //Create a configuration for configuring the pipeline with a non default profile
  LOG(INFO) << "Configure depth sensor (" << WID_DEPTH << "x" << HEI_DEPTH << "@" << FPS_DEPTH <<")";
  rs2::config cfg;

  //Add desired streams to configuration
  cfg.enable_stream(RS2_STREAM_INFRARED, WID_DEPTH, HEI_DEPTH, RS2_FORMAT_Y8, FPS_DEPTH);
  cfg.enable_stream(RS2_STREAM_DEPTH, WID_DEPTH, HEI_DEPTH, RS2_FORMAT_Z16, FPS_DEPTH);
  //Instruct pipeline to start streaming with the requested configuration
  pipe.start(cfg);

  // Camera warmup - dropping several first frames to let auto-exposure stabilize
  LOG(INFO) << "Start capturing...";
  rs2::frameset frames;
  while(true) {
    frames = pipe.wait_for_frames();
    //Get each frame
    rs2::frame ir_frame = frames.first(RS2_STREAM_INFRARED);
    rs2::frame depth_frame = frames.get_depth_frame();

    // Creating OpenCV matrix from IR image
    Mat ir(Size(WID_DEPTH, HEI_DEPTH), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);

    // Apply Histogram Equalization
    equalizeHist( ir, ir );
    applyColorMap(ir, ir, COLORMAP_JET);

    // Display the image in GUI
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", ir);

    if (waitKey(5) == 27) break;
  }

  return 0;
}
