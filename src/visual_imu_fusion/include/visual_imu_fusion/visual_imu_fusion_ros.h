/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

	@section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef VISUAL_IMU_FUSION_ROS_H
#define VISUAL_IMU_FUSION_ROS_H

#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include <std_msgs/Float32MultiArray.h>
#include <fstream>
#include <iostream>

#include "visual_imu_fusion/visual_imu_fusion.h"

namespace VINS_CCNY {

class ComplementaryFilterROS
{
  public:
    ComplementaryFilterROS(const ros::NodeHandle& nh, 
                           const ros::NodeHandle& nh_private);
    virtual ~ComplementaryFilterROS();

    std::ofstream myfile;

    int initialized;


  private:


    typedef image_transport::ImageTransport ImageTransport;
    typedef sensor_msgs::Image  ImageMsg;  // applicable for both Image and Depth
    typedef sensor_msgs::CameraInfo  CameraInfoMsg;
    typedef std_msgs::Float32MultiArray CovMsg;   // subsribe covariance message
    typedef nav_msgs::Odometry OdomMsg;           // subsribe motion message
    typedef nav_msgs::Path PathMsg;               // subsribe path message
    typedef geometry_msgs::PoseStamped Posemsg;

    typedef message_filters::Subscriber<ImageMsg> ImageSubscriber;
    typedef message_filters::Subscriber<CameraInfoMsg> CameraInforSubscriber;
    typedef message_filters::Subscriber<CovMsg> CovSubscriber;
    typedef message_filters::Subscriber<OdomMsg> MotionSubscriber;
    typedef message_filters::Subscriber<PathMsg> PathSubscriber;

    typedef message_filters::Subscriber<Posemsg> PoseSubscriber;

    boost::shared_ptr<ImageSubscriber> image_subscriber_;
    boost::shared_ptr<ImageSubscriber> rgbd_subscriber_;
    boost::shared_ptr<CameraInforSubscriber> camerainfor_subscriber_;
    boost::shared_ptr<CovSubscriber> cov_subscriber_;
    boost::shared_ptr<MotionSubscriber> motion_subscriber_;
    boost::shared_ptr<PathSubscriber>  path_subscriber_;
    boost::shared_ptr<PoseSubscriber>  pose_subscriber_;


    // Convenience typedefs
    typedef sensor_msgs::Imu ImuMsg;
    typedef sensor_msgs::MagneticField MagMsg;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, 
        MagMsg> MySyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> 
        SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;    
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber; 
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;

    // ROS-related variables.
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    boost::shared_ptr<Synchronizer> sync_;
    boost::shared_ptr<ImuSubscriber> imu_subscriber_;
    boost::shared_ptr<MagSubscriber> mag_subscriber_;

    ros::Publisher imu_publisher_;
    ros::Publisher rpy_publisher_;
    ros::Publisher state_publisher_;
    ros::Publisher path_pub_;                 ///< ROS publisher for the path
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Publisher vins_pose_Pub_;

    // progration parameters
    Eigen::MatrixX3d accleration_in_;
    Eigen::MatrixXd X_state_;
    Eigen::MatrixXd X_state_prediction_;
    Eigen::MatrixXd Q_ncov_;
    Eigen::MatrixXd n_imu_;
    Eigen::MatrixXd P_scov_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd G_;
    Eigen::MatrixXd TM_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd last_Velo_;
    Eigen::MatrixXd last_position_;

    double gravity_;
    double initial_ax;
    double initial_ay;
    double initial_az;
    double last_time;
    int imuCount;



    // Parameters:
    bool use_mag_;
    bool publish_tf_;
    bool reverse_tf_;
    double constant_dt_;
    bool publish_debug_topics_;
    std::string fixed_frame_;
    nav_msgs::Path path_msg_;

    geometry_msgs::PoseStamped pose_msg_;
    double lastHeight_;
    ros::Time lastTime_;


    // State variables:
    ComplementaryFilter filter_;
    ros::Time time_prev_;
    bool initialized_filter_;
    bool slamPoseInitialize_;

    bool initialized_acc_;
    bool initialized_velo_;

    void initializeParams();
    void imuCallback(const ImuMsg::ConstPtr& imu_msg);
    void motionCallback(const OdomMsg::ConstPtr& motion_msg_raw);
    void covCallback (const CovMsg::ConstPtr& cov_msg);
    void poseCallback(const Posemsg::ConstPtr& pose_msg_slam);
    void publishPath(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    // ax ay az: acceleration measurement
    void getMeasurement(
        double ax, double ay, double az,
        double& q0_guess, double& q1_guess, double& q2_guess, double& q3_guess);

    void quaternionMult(Eigen::MatrixXd qauternionL, Eigen::MatrixXd& qauternionR,
    		                                    Eigen::MatrixXd& multResult);
    ///////////////////////////////////////
    void accelerationCamera(double& ax, double& ay, double& az, Eigen::MatrixXd x_state);

    //
    tf::Transform tfFromEigen(Eigen::Matrix4f trans);


};

}  // namespace imu_tools

#endif // IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
