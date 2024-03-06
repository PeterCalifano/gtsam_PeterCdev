#include "DenseVis.h" 2 3 DenseVis::DenseVis(Triangulator* triangulator, LCMPublisher* _lcmpub){4 denseStereo = new DenseStereo(*triangulator, false); 5 lcmpub = _lcmpub; 6 7 cloud = pcl::PointCloud < pcl::PointXYZRGB> ::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>); 8 totalCloud = pcl::PointCloud < pcl::PointXYZRGB> ::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>); 9 10 smallTriangles = pcl::PolygonMeshPtr(new pcl::PolygonMesh); 11 totalTriangles = pcl::PolygonMeshPtr(new pcl::PolygonMesh); 12 13 14 } 15 16 void DenseVis::clear(){17 princAxisPoints.clear(); 18 princAxisColors.clear(); 19 } 20 21 void DenseVis::buildDenseMap(isam::cameraPose3d_Node * cam, isam::dynamicPose3d_NL_Node * princAxis, std::vector <isam::dynamicPose3d_NL_Node*> & poselist, std::vector <cv::Mat> & leftImageList, std::vector <cv::Mat> & rightImageList){22 23 int size = poselist.size(); 24 lcmpub->clearBody3DPoints(); 25 lcmpub->addPrincipalAxis(princAxis, poselist.size()); 26 for (int i = 0; i < size; i++){27 std::cout << "Dense Map Iteration: " << i << std::endl; 28 this->clear(); 29 computeDensePoints(cam, poselist[i], leftImageList[i], rightImageList[i]); 30 31 lcmpub->addBody3DPoints(i, princAxisPoints, princAxisColors); 32 usleep(100000); 33 } 34 } 35 36 void DenseVis::updatePrincipalAxis(isam::dynamicPose3d_NL_Node * princAxis, int listsize){37 lcmpub->addPrincipalAxis(princAxis, listsize); 38 }
void DenseVis::computeDensePoints(isam::cameraPose3d_Node* cam,
                                  isam::dynamicPose3d_NL_Node* pose,
                                  cv::Mat& leftImage,
                                  cv::Mat& rightImage) {
  41 double lx, ly, lz;
  42 isam::Point3dh X;
  43 isam::Point3dh inertX;
  44 isam::Point3dh bodyX;
  45 Eigen::Vector3d bodyVec;
  46 cv::equalizeHist(leftImage, leftImage);
  47 cv::equalizeHist(rightImage, rightImage);
  48 49 denseStereo->clear();
  50 51 elasDisp = denseStereo->calculate(leftImage, rightImage);
  52 nonthresholded_img = denseStereo->getPreThreshDisp();
  53 54 for (unsigned int i = 0; i < denseStereo->points.size(); i++) {
    55 lx = denseStereo->points[i](0);
    56 ly = denseStereo->points[i](1);
    57 lz = denseStereo->points[i](2);
    58 59 X.set(lz, -lx, -ly, 1.0);
    60 inertX = cam->value().transform_from(X);
    61 bodyX = pose->value().transform_to_body(inertX);
    62 princAxisPoints.push_back(Eigen::Vector3d(bodyX.x(), bodyX.y(), bodyX.z()));
    63 princAxisColors.push_back(denseStereo->colors[i]);
    64
  }
  65 66
}
67 68 void DenseVis::buildDenseCloud(isam::cameraPose3d_Node* cam,
                                     std::vector<isam::dynamicPose3d_NL_Node*>& poselist,
                                     std::vector<cv::Mat>& leftImageList,
                                     std::vector<cv::Mat>& rightImageList) {
  69 std::stringstream filename, filename2, filename3, filename4, filename5, filename6;
  70 denseStereo->clear();
  71 std::cout << "Poselist, left, right: " << poselist.size() << "," << leftImageList.size() << ","
               << rightImageList.size() << std::endl;
  72 73 for (int j = 0; j < poselist.size(); j++) {
    74 filename.str(std::string());
    75 filename2.str(std::string());
    76 filename3.str(std::string());
    77 filename4.str(std::string());
    78 filename5.str(std::string());
    79 filename6.str(std::string());
    80 this->clear();
    computeDensePoints(cam, poselist[j], leftImageList[j + 1], rightImageList[j + 1]);
    82 83 filename4 << "/home/tweddle/Desktop/disparity/mergeImg" << j << ".bmp";
    84 filename5 << "/home/tweddle/Desktop/disparity/elasDisp" << j << ".bmp";
    85 86 cv::Mat mergeImg;
    87 std::vector<cv::Mat> channels;
    88 channels.push_back(leftImageList[j + 1]);
    89 channels.push_back(leftImageList[j + 1]);
    90 channels.push_back(elasDisp);
    91 cv::merge(channels, mergeImg);
    92 93 94 cv::imwrite(filename4.str(), mergeImg);
    95 cv::imwrite(filename5.str(), elasDisp);
    96 97 std::cout << "PrincAxisPoints: " << princAxisPoints.size() << std::endl;
    98 99 cloud->points.clear();
    100 cloud->points.resize(princAxisPoints.size());
    101 cloud->width = cloud->size();
    102 cloud->height = 1;
    103 104  // totalCloud->points.resize(totalCloud->points.size() + princAxisPoints.size() ); 105
             // 106 for (unsigned int i = 0; i < princAxisPoints.size(); i++) { 107
             // cloud->points[i].x = princAxisPoints[i](0); 108 cloud->points[i].y =
             // princAxisPoints[i](1); 109 cloud->points[i].z = princAxisPoints[i](2); 110 111
             // uint8_t r = princAxisColors[i]; 112 uint8_t g = princAxisColors[i]; 113 uint8_t b =
             // princAxisColors[i]; // Example: Red color 114 uint32_t rgb = ((uint32_t)r << 16 |
             // (uint32_t)g << 8 | (uint32_t)b); 115 cloud->points[i].rgb =
             // *reinterpret_cast<float*>(&rgb); 116 117 pcl::PointXYZRGB newPoint; 118 newPoint.x =
             // princAxisPoints[i](0); 119 newPoint.y = princAxisPoints[i](1); 120 newPoint.z =
             // princAxisPoints[i](2); 121 newPoint.rgb = *reinterpret_cast<float*>(&rgb); 122 123
             // 124 totalCloud->points.push_back(newPoint);

        totalCloud->width = totalCloud->points.size();
    126 totalCloud->height = 1;
    127 128
  }
  129 filename << "/home/tweddle/Desktop/plyfolder/plyfile" << j << ".ply";
  130 filename2 << "/home/tweddle/Desktop/pcdfolder/pcdfile" << j << ".pcd";
  131 plyWriter.write(filename.str(), *(cloud.get()));
  132 133 std::cout << "Width/Height: " << cloud->width << "," << cloud->height << ","
                    << cloud->size() << "," << princAxisPoints.size() << std::endl;
  134 std::cout << "TotalCloud Width/Height: " << totalCloud->width << "," << totalCloud->height
                << "," << totalCloud->size() << std::endl;
  135 136 pcl::io::savePCDFile(filename2.str(), *(cloud.get()));
  137 138 filename3 << "/home/tweddle/Desktop/meshfolder/mesh" << j << ".vtk";
  139 std::cout << "filename3: " << filename3.str() << std::endl;
  140 141 142
}
143 144 std::cout << "totalCloud->points.size(): " << totalCloud->points.size() << std::endl;
145 146 pcl::PointCloud<pcl::PointXYZRGB>::Ptr totalSampleSmall =
    downsampleCloud(totalCloud, 0.001);
147 filename6 << "/home/tweddle/Desktop/meshfolder/totalmesh.vtk";
148 generateMesh(totalSampleSmall, totalTriangles, filename6.str(), 500);
149 150 pcl::io::savePolygonFileSTL("/home/tweddle/Desktop/meshfolder/totalMesh.stl",
                                    *totalTriangles);
151 pcl::io::savePLYFile("/home/tweddle/Desktop/meshfolder/totalMesh.ply", *totalTriangles, 6);
152 153 154
}
155 156 pcl::PointCloud<pcl::PointXYZRGB>::Ptr DenseVis::downsampleCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, float dimension) {
  157 std::cout << "Downsampling cloud - initial size: " << inputCloud->points.size() << std::endl;
  158 159 pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  160 pcl::PointCloud<pcl::PointXYZRGB>::Ptr returnCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  161 162 pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  163 sor.setInputCloud(inputCloud);
  164 sor.setMeanK(50);
  165 sor.setStddevMulThresh(1.0);
  166 sor.filter(*outputCloud);
  167 168 std::cout << "SOR completed - final size: " << outputCloud->points.size() << std ::endl;
  169 170  // Create the filtering object 171 pcl::VoxelGrid<pcl::PointXYZRGB> vgrid; 172
           // vgrid.setInputCloud(outputCloud); 173 vgrid.setLeafSize(dimension, dimension,
           // dimension); 174 vgrid.filter(*returnCloud); 175 176 std::cout << "Downsample completed
           // - final size: " << returnCloud->points.size() << std::endl; 177 178 return
           // returnCloud; 179 } 180 181 void
           // DenseVis::generateMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::
           // PolygonMesh::Ptr triangles, std::string filename, int maxNN) { 182 183 // Normal
           // estimation* 184 pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n; 185
           // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); 186
           // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::
           // PointXYZRGB>); 187 tree->setInputCloud (cloud); 188 n.setInputCloud (cloud); 189
           // n.setSearchMethod (tree); 190 n.setKSearch (20); 191 n.compute (*normals); 192 //*
           // normals should not contain the point normals + surface curvatures 193 194 //
           // Concatenate the XYZ and normal fields* 195
           // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::
           // PointCloud<pcl::PointXYZRGBNormal>); 196 pcl::concatenateFields (*cloud, *normals,
           // *cloud_with_normals); 197 198 std::cout << "Point A\n";
      // Create search tree* 201 pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new
      // pcl::search::KdTree< pcl::PointXYZRGBNormal>); 202 tree2->setInputCloud
      // (cloud_with_normals); 203 204 std::cout << "Point B\n"; 205 206 // Initialize objects 207
      // pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3; 208 209 std::cout << "Point
      // C\n"; 210 211 // Set typical values for the parameters 212 gp3.setMu (2.5); 213
      // gp3.setSearchRadius(0.05); 214 gp3.setMaximumNearestNeighbors (maxNN); 215
      // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees 216 gp3.setMinimumAngle(M_PI/18); // 10
      // degrees 217 gp3.setMaximumAngle(2*M_PI/3); // 120 degrees 218
      // gp3.setNormalConsistency(true); 219 220 221 // Get result 222 gp3.setInputCloud
      // (cloud_with_normals); 223 gp3.setSearchMethod (tree2); 224 std::cout << "Point D:
      // cloudnormals: " << cloud_with_normals->size() << std:: endl; 225 std::cout << "num of
      // triangles1: " << triangles->polygons.size() << std::endl; 226 std::cout << "search radius:
      // " << gp3.getSearchRadius() << std::endl;; 227 228 gp3.reconstruct(*triangles); 229 230
      // std::cout << "Point E\n"; 231 232 // Additional vertex information 233 std::vector<int>
      // parts = gp3.getPartIDs(); 234 std::vector<int> states = gp3.getPointStates(); 235 236
      // std::cout << "num of triangles: " << triangles->polygons.size()
      // << std::endl; 237 std::cout << "write filename: " << filename << std::endl; 238
      // pcl::io::saveVTKFile(filename, *triangles); 239 240 } 241 242 void
      // DenseVis::visualizeMesh(pcl::PolygonMeshPtr mesh, std::vector<isam::dynamicPose3d_NL_Node*>
      // pose_list, std::vector<cv::Mat>& leftImageList) { 243 Eigen::Vector3f offset; 244
      // Eigen::Vector4f offset4; 245 Eigen::Quaternionf quat; 246 Eigen::Vector4d currq; 247
      // Eigen::Vector3d currp; 248 std::stringstream visFilename; 249 std::stringstream
      // leftFilename; 250 251 pcl::visualization::PCLVisualizer vis("Mesh Viewer"); 252
      // //pcl::visualization::ImageViewer iv("Left Camera Viewer"); 253 254 cv::Mat combined_img;
      // 255
      // //place two images side by side 256 combined_img.create( cv::Size(2*640,480),
      // CV_MAKETYPE(leftImageList[0].depth(), 3) ); 257 cv::Mat imgLeft = combined_img( cv::Rect(0,
      // 0, leftImageList[0].cols, leftImageList[0].rows)); 258 cv::Mat imgRight = combined_img(
      // cv::Rect(leftImageList[0].cols, 0, leftImageList [0].cols, leftImageList[0].rows) ); 259
      // 260 261 std::cout << "Visualizing Mesh" << std::endl; 262 263 offset << -0.35, 0, 0; 264
      // quat.x() = 0.0; 265 quat.y() = 0.0; 266 quat.z() = 0.0; 267 quat.w() = 1.0; 268 269
      // Eigen::Affine3f affineTransform = Eigen::Translation3f(offset) * Eigen::AngleAxisf (quat);
      // 270 271 vis.setBackgroundColor(0.1, 0.01,1.0); 272 vis.addPolygonMesh(*mesh); 273
      // vis.initCameraParameters(); 274 vis.setCameraPosition(-0.5,0,0, 0,0,1); 275
      // vis.spinOnce(100); 276 277 278 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr meshcloud(new
      // pcl::PointCloud<pcl:: PointXYZRGBNormal>); 279 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
      // temp2(new pcl::PointCloud<pcl:: PointXYZRGBNormal>); 280 pcl::fromROSMsg(mesh->cloud,
      // *meshcloud);

      std::cout
      << "About to spin" << std::endl;
  284 285 for (unsigned int i = 0; i < pose_list.size(); i++) {
    286 if (vis.wasStopped()) {
      287 break;
      288
    }
    289 290 currp = pose_list[i]->value().x().head(3);
    291 currq = pose_list[i]->value().qTotal();
    292 offset(0) = currp(0);
    293 offset(1) = currp(1);
    294 offset(2) = currp(2);
    295 offset4.head(3) = offset;
    296 offset4(3) = 0.0;
    297 quat.x() = currq(0);
    298 quat.y() = currq(1);
    299 quat.z() = currq(2);
    300 quat.w() = currq(3);
    301 302 affineTransform = Eigen::Translation3f(offset) * Eigen::AngleAxisf(quat);
    303 304 std::cout << "Counter: " << i << std::endl;
    305 306 pcl::transformPointCloudWithNormals(*meshcloud, *temp2, offset, quat);
    307 vis.updatePolygonMesh<pcl::PointXYZRGBNormal>(temp2, mesh->polygons);
    308 visFilename.str(std::string());
    309 visFilename << "/home/tweddle/Desktop/visfolder/visfile" << i << ".png";
    310 vis.saveScreenshot(visFilename.str());
    311 312 cv::Mat tempVis = cv::imread(visFilename.str());
    313 cv::resize(tempVis, imgRight, cv::Size(640, 480));
    314 cvtColor(leftImageList[i + 1], imgLeft, CV_GRAY2BGR);
    315 316 leftFilename.str(std::string());
    317 leftFilename << "/home/tweddle/Desktop/visfolder/combinedImg" << i << ".png";
    318 cv::imwrite(leftFilename.str(), combined_img);
    319 320 321  // 500 ms with 1ms draw every 50 ms 322 for (int j = 0; j < 10; j++) { 323
                 // vis.spinOnce(100); 324 usleep(50000); 325 }} 328 329 //5000 ms with 1ms draw
                 // every 50 ms 330 while(!vis.wasStopped()) { 331 vis.spinOnce(100); 332
                 // usleep(100000); 333 } 334 335 std::cout << "Done with spin" << std::endl; 336 }