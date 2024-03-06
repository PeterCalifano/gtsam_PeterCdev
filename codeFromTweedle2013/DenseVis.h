/* 2 * DenseVis.h 3* 4 * Created on: May 5, 2013 5 * Author: tweddle 6 */
    7 8 #ifndef DENSEVIS_H_ 9 #define DENSEVIS_H_ 10 11 #include < fstream > 12 #include < vector >
    13 #include < string > 14 #include < iostream > 15 #include < sstream > 16 #include <
    exception >
    17 18  // Eigen 19 #include <Eigen/Core> 20 #include <Eigen/Geometry> 21 #include
           // <Eigen/StdVector> 22 23 // OpenCV 24 #include "opencv2/core/core.hpp" 25 #include
           // "opencv2/imgproc/imgproc.hpp" 26 #include "opencv2/calib3d/calib3d.hpp" 27 #include
           // "opencv2/highgui/highgui.hpp" 28 29 // libelas 30 #include "elas.h" 31 32 #include
           // "Triangulator.h" 33 #include "DenseStereo.h" 34 //#include "Frame.h" 35 36 #include
           // "LCMPublisher.h" 37 38 39 #include <pcl/point_types.h> 40 #include <pcl/io/ply_io.h>
           // 41 #include <pcl/io/pcd_io.h> 42 #include <pcl/kdtree/kdtree_flann.h> 43 #include
           // <pcl/features/normal_3d.h> 44 #include <pcl/surface/gp3.h>

#include <pcl/io/vtk_io.h> 46 #include <pcl/io/vtk_lib_io.h> 47 #include <pcl/filters/voxel_grid.h> 48 #include <pcl/filters/statistical_outlier_removal.h> 49 #include <pcl/visualization/pcl_visualizer.h> 50 #include <pcl/common/transforms.h> 51 #include <pcl/visualization/image_viewer.h> 52 53 class DenseVis { 54 cv::Mat elasDisp, nonthresholded_img; 55 DenseStereo* denseStereo; 56 LCMPublisher* lcmpub; 57 58 std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > princAxisPoints; 59 std::vector<int> princAxisColors; 60 61 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, totalCloud; 62 pcl::PLYWriter plyWriter; 63 64 public: 65 66 pcl::PolygonMeshPtr smallTriangles; 67 pcl::PolygonMeshPtr totalTriangles; 68 69 DenseVis(Triangulator* triangulator, LCMPublisher* _lcmpub); 70 71 void computeDensePoints(isam::cameraPose3d_Node* cam, isam::dynamicPose3d_NL_Node* pose, cv::Mat& leftImage, cv::Mat& rightImage); 72 73 void buildDenseMap(isam::cameraPose3d_Node* cam, isam::dynamicPose3d_NL_Node* princAxis, std::vector<isam::dynamicPose3d_NL_Node*>& poselist, std::vector<cv ::Mat>& leftImageList, std::vector<cv::Mat>& rightImageList); 74 void buildDenseCloud(isam::cameraPose3d_Node* cam, /*isam::dynamicPose3d_NL_Node* princAxis,*/std::vector<isam::dynamicPose3d_NL_Node*>& poselist, std::vector< cv::Mat>& leftImageList, std::vector<cv::Mat>& rightImageList); 75 void updatePrincipalAxis(isam::dynamicPose3d_NL_Node* princAxis, int listsize); 76 void generateMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl:: PolygonMeshPtr triangles, std::string filename, int maxNN = 400); 77 pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampleCloud(pcl::PointCloud<pcl:: PointXYZRGB>::Ptr cloud, float dimension); 78 void visualizeMesh(pcl::PolygonMeshPtr mesh, std::vector<isam:: dynamicPose3d_NL_Node*> pose_list, std::vector<cv::Mat>& leftImageList); 79 80 void clear();
    }
    ;
    83 84 #endif /* DENSEVIS_H_ */