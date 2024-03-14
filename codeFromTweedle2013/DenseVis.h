/*  DenseVis.h
 Created on: May 5, 2013
  Author: tweddle  */
#ifndef DENSEVIS_H_
#define DENSEVIS_H_
#endif

// STL
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// libelas
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h >
#include <pcl/io/ply_io.h >
#include <pcl/kdtree/kdtree_flann.h >
#include "DenseStereo.h"
#include "Frame.h"
#include "LCMPublisher.h"
#include "Triangulator.h"
#include "elas.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

class DenseVis {
  cv::Mat elasDisp, nonthresholded_img;
  DenseStereo* denseStereo;
  LCMPublisher* lcmpub;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > princAxisPoints;

  std::vector<int> princAxisColors;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, totalCloud;
  pcl::PLYWriter plyWriter;

 public:
  pcl::PolygonMeshPtr smallTriangles;
  pcl::PolygonMeshPtr totalTriangles;
  DenseVis(Triangulator* triangulator, LCMPublisher* _lcmpub);
  void computeDensePoints(isam::cameraPose3d_Node* cam,
                          isam::dynamicPose3d_NL_Node* pose,
                          cv::Mat& leftImage,
                          cv::Mat& rightImage);
  void buildDenseMap(isam::cameraPose3d_Node* cam,
                     isam::dynamicPose3d_NL_Node* princAxis,
                     std::vector<isam::dynamicPose3d_NL_Node*>& poselist,
                     std::vector<cv ::Mat>& leftImageList,
                     std::vector<cv::Mat>& rightImageList);
  void buildDenseCloud(isam::cameraPose3d_Node* cam,
                       /*isam::dynamicPose3d_NL_Node* princAxis,*/
                       std::vector<isam::dynamicPose3d_NL_Node*>& poselist,
                       std::vector<cv::Mat>& leftImageList,
                       std::vector<cv::Mat>& rightImageList);
  void updatePrincipalAxis(isam::dynamicPose3d_NL_Node* princAxis, int listsize);
  void generateMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PolygonMeshPtr triangles,
                    std::string filename,
                    int maxNN = 400);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampleCloud(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float dimension);
  void visualizeMesh(pcl::PolygonMeshPtr mesh,
                     std::vector<isam::dynamicPose3d_NL_Node*> pose_list,
                     std::vector<cv::Mat>& leftImageList);
  void clear();
};
#endif /* DENSEVIS_H_ */