#include "DenseVis.h"
DenseVis::DenseVis(Triangulator* triangulator, LCMPublisher* _lcmpub) {
  denseStereo = new DenseStereo(*triangulator, false);
  lcmpub = _lcmpub;
  cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  totalCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  smallTriangles = pcl::PolygonMeshPtr(new pcl::PolygonMesh);
  totalTriangles = pcl::PolygonMeshPtr(new pcl::PolygonMesh);
}
void DenseVis::clear() {
  princAxisPoints.clear();
  princAxisColors.clear();
}
void DenseVis::buildDenseMap(isam::cameraPose3d_Node* cam,
                             isam::dynamicPose3d_NL_Node* princAxis,
                             std::vector<isam::dynamicPose3d_NL_Node*>& poselist,
                             std::vector<cv::Mat>& leftImageList,
                             std::vector<cv::Mat>& rightImageList) {
  int size = poselist.size();
  lcmpub->clearBody3DPoints();
  lcmpub->addPrincipalAxis(princAxis, poselist.size());
  for (int i = 0; i < size; i++) {
    std::cout << "Dense Map Iteration: " << i << std::endl;
    this->clear();
    computeDensePoints(cam, poselist[i], leftImageList[i], rightImageList[i]);
    lcmpub->addBody3DPoints(i, princAxisPoints, princAxisColors);
    usleep(100000);
  }
}
void DenseVis::updatePrincipalAxis(isam::dynamicPose3d_NL_Node* princAxis, int listsize) {
  lcmpub->addPrincipalAxis(princAxis, listsize);
}
void DenseVis::computeDensePoints(isam::cameraPose3d_Node* cam,
                                  isam::dynamicPose3d_NL_Node* pose,
                                  cv::Mat& leftImage,
                                  cv::Mat& rightImage) {
  double lx, ly, lz;
  isam::Point3dh X;
  isam::Point3dh inertX;
  isam::Point3dh bodyX;
  Eigen::Vector3d bodyVec;
  cv::equalizeHist(leftImage, leftImage);
  cv::equalizeHist(rightImage, rightImage);
  denseStereo->clear();
  elasDisp = denseStereo->calculate(leftImage, rightImage);
  nonthresholded_img = denseStereo->getPreThreshDisp();
  for (unsigned int i = 0; i < denseStereo->points.size(); i++) {
    lx = denseStereo->points[i](0);
    ly = denseStereo->points[i](1);
    lz = denseStereo->points[i](2);
    X.set(lz, -lx, -ly, 1.0);
    inertX = cam->value().transform_from(X);
    bodyX = pose->value().transform_to_body(inertX);
    princAxisPoints.push_back(Eigen::Vector3d(bodyX.x(), bodyX.y(), bodyX.z()));
    princAxisColors.push_back(denseStereo->colors[i]);
  }
}
void DenseVis::buildDenseCloud(isam::cameraPose3d_Node* cam,
                               std::vector<isam::dynamicPose3d_NL_Node*>& poselist,
                               std::vector<cv::Mat>& leftImageList,
                               std::vector<cv::Mat>& rightImageList) {
  std::stringstream filename, filename2, filename3, filename4, filename5, filename6;
  denseStereo->clear();
  std::cout << "Poselist, left, right: " << poselist.size() << "," << leftImageList.size() << ","
            << rightImageList.size() << std::endl;
  for (int j = 0; j < poselist.size(); j++) {
    filename.str(std::string());
    filename2.str(std::string());
    filename3.str(std::string());
    filename4.str(std::string());
    filename5.str(std::string());
    filename6.str(std::string());
    this->clear();
    computeDensePoints(cam, poselist[j], leftImageList[j + 1], rightImageList[j + 1]);
    filename4 << "/home/tweddle/Desktop/disparity/mergeImg" << j << ".bmp"; // REPLACE PATH
    filename5 << "/home/tweddle/Desktop/disparity/elasDisp" << j << ".bmp";
    cv::Mat mergeImg;
    std::vector<cv::Mat> channels;
    channels.push_back(leftImageList[j + 1]);
    channels.push_back(leftImageList[j + 1]);
    channels.push_back(elasDisp);
    cv::merge(channels, mergeImg);
    cv::imwrite(filename4.str(), mergeImg);
    cv::imwrite(filename5.str(), elasDisp);
    std::cout << "PrincAxisPoints: " << princAxisPoints.size() << std::endl;
    cloud->points.clear();
    cloud->points.resize(princAxisPoints.size());
    cloud->width = cloud->size();
    cloud->height = 1;
    totalCloud->points.resize(totalCloud->points.size() + princAxisPoints.size());
    for (unsigned int i = 0; i < princAxisPoints.size(); i++) {
      cloud->points[i].x = princAxisPoints[i](0);
      cloud->points[i].y = princAxisPoints[i](1);
      cloud->points[i].z = princAxisPoints[i](2);
      uint8_t r = princAxisColors[i];
      uint8_t g = princAxisColors[i];
      uint8_t b = princAxisColors[i];  // Example: Red color
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b); // Cast colours to uint32
      cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
      pcl::PointXYZRGB newPoint;
      newPoint.x = princAxisPoints[i](0);
      newPoint.y = princAxisPoints[i](1);
      newPoint.z = princAxisPoints[i](2);
      newPoint.rgb = *reinterpret_cast<float*>(&rgb);
      totalCloud->points.push_back(newPoint) totalCloud->width = totalCloud->points.size();
      totalCloud->height = 1;
    }
    filename << "/home/tweddle/Desktop/plyfolder/plyfile" << j << ".ply";
    filename2 << "/home/tweddle/Desktop/pcdfolder/pcdfile" << j << ".pcd";
    plyWriter.write(filename.str(), *(cloud.get()));
    std::cout << "Width/Height: " << cloud->width << "," << cloud->height << "," << cloud->size()
              << "," << princAxisPoints.size() << std::endl;
    std::cout << "TotalCloud Width/Height: " << totalCloud->width << "," << totalCloud->height
              << "," << totalCloud->size() << std::endl;
    pcl::io::savePCDFile(filename2.str(), *(cloud.get()));
    filename3 << "/home/tweddle/Desktop/meshfolder/mesh" << j << ".vtk";
    std::cout << "filename3: " << filename3.str() << std::endl;
  }
  std::cout << "totalCloud->points.size(): " << totalCloud->points.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr totalSampleSmall = downsampleCloud(totalCloud, 0.001);
  filename6 << "/home/tweddle/Desktop/meshfolder/totalmesh.vtk";
  generateMesh(totalSampleSmall, totalTriangles, filename6.str(), 500);
  pcl::io::savePolygonFileSTL("/home/tweddle/Desktop/meshfolder/totalMesh.stl", *totalTriangles);
  pcl::io::savePLYFile("/home/tweddle/Desktop/meshfolder/totalMesh.ply", *totalTriangles, 6);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr DenseVis::downsampleCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, float dimension) {
  std::cout << "Downsampling cloud - initial size: " << inputCloud->points.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr returnCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(inputCloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*outputCloud);
  std::cout << "SOR completed - final size: " << outputCloud->points.size() << std ::endl;
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> vgrid;

  vgrid.setInputCloud(outputCloud);
  vgrid.setLeafSize(dimension, dimension, dimension);
  vgrid.filter(*returnCloud);
  std::cout << "Downsample completed
                   - final size : " << returnCloud->points.size() << std::endl;  return
                                  returnCloud;
}
void DenseVis::generateMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            pcl::PolygonMesh::Ptr triangles,
                            std::string filename,
                            int maxNN) {
  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);
  // normals should not contain the point normals + surface curvatures
  // Concatenate the XYZ and
  // normal fields
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
  std::cout << "Point A\n";
  // Create search tree *
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud(cloud_with_normals);
  std::cout << "Point B\n";
  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  std::cout << "Point
               C\n "; 
               // Set typical values for the parameters 212 gp3.setMu (2.5);
               gp3.setSearchRadius(0.05);
  gp3.setMaximumNearestNeighbors(maxNN);
  gp3.setMaximumSurfaceAngle(M_PI / 4);       // 45 degrees
  gp3.setMinimumAngle(M_PI / 18);             // 10
  degrees gp3.setMaximumAngle(2 * M_PI / 3);  // 120 degrees
  gp3.setNormalConsistency(true);
  // Get result
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  std::cout << "Point D:
      cloudnormals
      : " << cloud_with_normals->size() << std:: endl;  std::cout << " num of triangles1
      : " << triangles->polygons.size() << std::endl;  std::cout << " search radius
      : " << gp3.getSearchRadius() << std::endl;  gp3.reconstruct(*triangles); 
        std::cout
            << "Point E\n";
  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  std::cout << "num of triangles: " << triangles->polygons.size() << std::endl;
  std::cout << "write filename: " << filename << std::endl;
  pcl::io::saveVTKFile(filename, *triangles);
}
void DenseVis::visualizeMesh(pcl::PolygonMeshPtr mesh,
                             std::vector<isam::dynamicPose3d_NL_Node*> pose_list,
                             std::vector<cv::Mat>& leftImageList) {
  Eigen::Vector3f offset;
  Eigen::Vector4f offset4;
  Eigen::Quaternionf quat;
  Eigen::Vector4d currq;
  Eigen::Vector3d currp;
  std::stringstream visFilename;
  std::stringstream leftFilename;
  pcl::visualization::PCLVisualizer vis("Mesh Viewer");
  pcl::visualization::ImageViewer iv("Left Camera Viewer");
  cv::Mat combined_img;

  // place two images side by side 256 combined_img.create( cv::Size(2*640,480),
   CV_MAKETYPE(leftImageList[0].depth(), 3) );
   cv::Mat imgLeft = combined_img(cv::Rect(0, 0, leftImageList[0].cols, leftImageList[0].rows));
   cv::Mat imgRight = combined_img(
       cv::Rect(leftImageList[0].cols, 0, leftImageList[0].cols, leftImageList[0].rows));
   std::cout << "Visualizing Mesh" << std::endl;
   offset << -0.35, 0, 0;
   quat.x() = 0.0;
   quat.y() = 0.0;
   quat.z() = 0.0;
   quat.w() = 1.0;
   Eigen::Affine3f affineTransform = Eigen::Translation3f(offset) * Eigen::AngleAxisf(quat);
   vis.setBackgroundColor(0.1, 0.01, 1.0);
   vis.addPolygonMesh(*mesh);
   vis.initCameraParameters();
   vis.setCameraPosition(-0.5, 0, 0, 0, 0, 1);
   vis.spinOnce(100);
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr meshcloud(
       new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   pcl::fromROSMsg(mesh->cloud, *meshcloud);

   std::cout << "About to spin" << std::endl;
   for (unsigned int i = 0; i < pose_list.size(); i++) {
     if (vis.wasStopped()) {
       break;
     }
     currp = pose_list[i]->value().x().head(3);
     currq = pose_list[i]->value().qTotal();
     offset(0) = currp(0);
     offset(1) = currp(1);
     offset(2) = currp(2);
     offset4.head(3) = offset;
     offset4(3) = 0.0;
     quat.x() = currq(0);
     quat.y() = currq(1);
     quat.z() = currq(2);
     quat.w() = currq(3);
     affineTransform = Eigen::Translation3f(offset) * Eigen::AngleAxisf(quat);
     std::cout << "Counter: " << i << std::endl;
     pcl::transformPointCloudWithNormals(*meshcloud, *temp2, offset, quat);
     vis.updatePolygonMesh<pcl::PointXYZRGBNormal>(temp2, mesh->polygons);
     visFilename.str(std::string());
     visFilename << "/home/tweddle/Desktop/visfolder/visfile" << i << ".png";
     vis.saveScreenshot(visFilename.str());
     cv::Mat tempVis = cv::imread(visFilename.str());
     cv::resize(tempVis, imgRight, cv::Size(640, 480));
     cvtColor(leftImageList[i + 1], imgLeft, CV_GRAY2BGR);
     leftFilename.str(std::string());
     leftFilename << "/home/tweddle/Desktop/visfolder/combinedImg" << i << ".png";
     cv::imwrite(leftFilename.str(), combined_img);
     // 500 ms with 1ms draw every 50 ms
     for (int j = 0; j < 10; j++) {
       vis.spinOnce(100);
       usleep(50000);
     }
   }
   // 5000 ms with 1ms draw every 50 ms
   while (!vis.wasStopped()) {
     vis.spinOnce(100);
     usleep(100000);
   }
   std::cout << "Done with spin" << std::endl;
}