#include <glog/logging.h>
#include <gflags/gflags.h>

#include <MappViewer.h>
#include <vsys.h>

#include "RsD4xx.h"

int main(int argc, char *argv[])
{
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // MappViewer
  std::unique_ptr<v::gview::MappViewer> mpv(new v::gview::MappViewer("../test/config/GLparameter.yaml"));
  // Start capture
  RsD4xx rs(RsD4xx::MODE_480x270_60HZ);
  bool bInit = rs.init();
  if (!bInit) { LOG(ERROR) << "No rs device is detected!"; return -1;}
  v::PointCloud::Ptr pc(new v::PointCloud);
  while(mpv->is_running()) {
    if (rs.proceed()) {
      rs.copyToSync(pc);
      // Display points
      v::ViewInType vit;
      vit.enOneLaserScan = 1;
      vit.oneLaserScan = pc;
      mpv->pipe_->enqueue_or_timeout(vit, 5);
      // check polling by setting timer
      // v::chrono::sleep_once(100);
    }
  }

  return 0;
}