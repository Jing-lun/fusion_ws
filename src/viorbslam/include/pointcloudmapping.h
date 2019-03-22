/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping( double resolution_ , const int sensor);

    // insert keyframe，update map
    void insertRGBDKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void insertStereoKeyFrame(KeyFrame* kf, cv::Mat& color, pcl::PCLPointCloud2& pointCloud2Dat);
    void shutdown();
    void viewer();
    void savePointCloudToPcd();

protected:
    PointCloud::Ptr generatePointCloudFromRGBD(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    PointCloud::Ptr globalMap;
    PointCloud::Ptr tempMap;
    shared_ptr<thread>  viewerThread;

    bool    shutDownFlag    =false;
    mutex   shutDownMutex;

    // Input sensor
    int mSensor;

    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;

    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    vector<pcl::PCLPointCloud2> pointCloud2Data;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;

    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel;

    //
    pcl::StatisticalOutlierRemoval<PointT> sor;

    // smoothing
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;

    // Output has the PointNormal type in order to store the normals calculated by MLS
      pcl::PointCloud<pcl::PointNormal> mls_points;

      // Create a KD-Tree
      pcl::search::KdTree<PointT>::Ptr tree;

};

#endif // POINTCLOUDMAPPING_H
