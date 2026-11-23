// No-op replacements for the four upstream viewer functions that registration.cpp references.
//
// Upstream defines them in visualization.cpp, which needs VTK and opens windows. We never call
// the two upstream registration() entry points that use them; these stubs exist only so the
// static library links in a headless build. Signatures are copied from Eva.h.
#include "Eva.h"

void visualization(PointCloudPtr /*cloud_src*/, PointCloudPtr /*cloud_tar*/, Eigen::Matrix4d /*Mat*/,
                   float /*resolution*/) {}

void Corres_Viewer_Score(PointCloudPtr /*cloud_s*/, PointCloudPtr /*cloud_t*/,
                         vector<Corre_3DMatch>& /*Hist_match*/, float& /*mr*/, int /*k*/) {}

void Corres_selected_visual(pcl::PointCloud<pcl::PointXYZ>::Ptr /*cloud_s*/,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr /*cloud_t*/,
                            vector<Corre_3DMatch>& /*Hist_match*/, float& /*mr*/, float /*GT_thresh*/,
                            Eigen::Matrix4d& /*GT_Mat*/) {}

float RMSE_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr /*cloud_source*/,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr /*cloud_target*/, Eigen::Matrix4d& /*Mat_est*/,
                   Eigen::Matrix4d& /*Mat_GT*/, float /*mr*/)
{
  return 0.0f;
}
