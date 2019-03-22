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
#include <iostream>
#include <string>
#include <ctime>
using namespace std;

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <tf/transform_datatypes.h>
#include <pcl/io/pcd_io.h>
#include "Converter.h"


#include <boost/make_shared.hpp>

PointCloudMapping::PointCloudMapping(double resolution_, const int sensor)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);


    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    mls.setComputeNormals (true);


    globalMap = boost::make_shared< PointCloud >( );
    tempMap   = boost::make_shared< PointCloud >( );

    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
    mSensor = sensor;
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertRGBDKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

void PointCloudMapping::insertStereoKeyFrame(KeyFrame* kf, cv::Mat& color, pcl::PCLPointCloud2& pointCloud2Dat)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    pointCloud2Data.push_back( pointCloud2Dat);

    keyFrameUpdated.notify_one();
}


pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloudFromRGBD(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    cv::Mat color_rgb;
    cv::cvtColor(color, color_rgb, CV_RGB2BGR);
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=5 )
    {
        for ( int n=0; n<depth.cols; n+=5 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;

            p.b = color_rgb.ptr<uchar>(m)[n*3];
            p.g = color_rgb.ptr<uchar>(m)[n*3+1];
            p.r = color_rgb.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    cv::Mat kfPose = kf->GetPose();

    cv::Mat Rwc = kfPose.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*kfPose.rowRange(0,3).col(3);

    tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
              Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
              Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));

    tf::Matrix3x3  inverRot(1.0, 0.0, 0.0,
           		      0.0, 1.0, 0.0,
           		      0.0, 0.0, 1.0 );

    //M = M*inverRot;

    tf::Vector3 V(twc.at<float>(0), -twc.at<float>(1), twc.at<float>(2));

    tf::Transform tfTcw(M,V);
    //kfPose.row(0).col(3) = twc.at<float>(0);
    //kfPose.row(1).col(3) = -twc.at<float>(1);
    //kfPose.row(2).col(3) = twc.at<float>(2);

   // kfPose.row(0).col(0) = M[0][0];
   // kfPose.row(0).col(1) = M[0][1];
   // kfPose.row(0).col(2) = M[0][2];
   // kfPose.row(1).col(0) = M[1][0];
   // kfPose.row(1).col(1) = M[1][1];
   // kfPose.row(1).col(2) = M[1][2];
   // kfPose.row(2).col(0) = M[2][0];
   // kfPose.row(2).col(1) = M[2][1];
   // kfPose.row(2).col(2) = M[2][2];

    //cv::Mat finalMatrix;
    //kfPose.row(0).col(0) = tfTcw[0][0];
    //kfPose.row(0).col(1) = tfTcw[0][1];
    //kfPose.row(0).col(2) = tfTcw[0][2];
    //kfPose.row(0).col(3) = tfTcw[0][3];
    //kfPose.row(1).col(0) = tfTcw[1][0];
    //kfPose.row(1).col(1) = tfTcw[1][1];
    //kfPose.row(1).col(2) = tfTcw[1][2];
    //kfPose.row(1).col(3) = tfTcw[1][3];
    //kfPose.row(2).col(0) = tfTcw[2][0];
    //kfPose.row(2).col(1) = tfTcw[2][1];
    //kfPose.row(2).col(2) = tfTcw[2][2];
    //kfPose.row(2).col(3) = tfTcw[2][3];
   // kfPose.row(3).col(0) = tfTcw[3][0];
   // kfPose.row(3).col(1) = tfTcw[3][1];
   // kfPose.row(3).col(2) = tfTcw[3][2];
   // kfPose.row(3).col(3) = tfTcw[3][3];




    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kfPose );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    clock_t end;
    clock_t begin;
    double elapsed_secs;
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {

        	begin = clock();
        	if (mSensor == System::RGBD){
                PointCloud::Ptr p = generatePointCloudFromRGBD( keyframes[i], colorImgs[i], depthImgs[i] );
        	    *globalMap += *p;
        	}

        	if (mSensor == System::STEREO){
        		PointCloud::Ptr p (new PointCloud);
        		pcl::fromPCLPointCloud2 (pointCloud2Data[i], *p);
        		*globalMap += *p;
        	}

            end = clock();
   	        elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
   	        cout<<"semi_dense elapse time: ->>>> "<<elapsed_secs<<std::endl;

        }
        PointCloud::Ptr tmp(new PointCloud());

        // StatisticalOutlierRemoval filter
        //sor.setInputCloud (globalMap);
        //sor.filter (*tmp);

        //tempMap->swap( *tmp );

        // voxel filter processing
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );


        globalMap->swap( *tmp );

        // perform smoothing
        mls.setInputCloud (globalMap);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.03);

        // Reconstruct
        mls.process (mls_points);

        pcl::copyPointCloud(mls_points, *tmp);

        viewer.showCloud( globalMap );
        cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;
    }
}

void PointCloudMapping::savePointCloudToPcd()
{
	    // set parameter
	    cout<<"Start saving the cloud."<<endl;
	    globalMap->height = 1;
	    globalMap->width = globalMap->points.size();
	    cout<<"point cloud size = "<<globalMap->points.size()<<endl;
	    globalMap->is_dense = false;
	    pcl::io::savePCDFile( "/home/ericyanng/map.pcd", *globalMap);
	    // clear data and exit
	    globalMap->points.clear();
	    cout<<"Point cloud saved."<<endl;
}

