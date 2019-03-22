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

#include "visual_imu_fusion/visual_imu_fusion_ros.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <stdio.h>
#include <stdlib.h>


namespace VINS_CCNY {

ComplementaryFilterROS::ComplementaryFilterROS(
    const ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  initialized_filter_(false),
  slamPoseInitialize_(false),
  initialized_acc_(false),
  initialized_velo_(false),
  initialized(0)
{
  ROS_INFO("Starting Visual Imu Fusion");
  initializeParams();
  
  int queue_size = 5;

  myfile.open ("/home/ericyang/ROS_WORKSPACE/visual_imu_fusion/pose/test1.txt");


  //Publish the path
  path_pub_ = nh_.advertise<nav_msgs::Path >("/path", queue_size);


  //pose subscribe
  pose_subscriber_.reset(new PoseSubscriber(nh_, "/AR_camera_pose", queue_size));
  pose_subscriber_->registerCallback( &ComplementaryFilterROS::poseCallback, this);

}

ComplementaryFilterROS::~ComplementaryFilterROS()
{
  ROS_INFO("Destroying ComplementaryFilterROS");
}

void ComplementaryFilterROS::initializeParams()
{
  double gain_acc;
  double gain_mag;  
  bool do_bias_estimation;
  double bias_alpha;
  bool do_adaptive_gain;

}

void ComplementaryFilterROS::covCallback (const CovMsg::ConstPtr& cov_msg)
{
	ROS_WARN("No processing in Covariance part.");
}

void ComplementaryFilterROS::poseCallback(const Posemsg::ConstPtr& pose_msg_slam)
{
	geometry_msgs::PoseStamped poseSlam = *pose_msg_slam;

	int testNumer = 0;

	if (initialized == 1)
	{
		myfile << poseSlam.pose.position.x << " " << poseSlam.pose.position.y << " " << poseSlam.pose.position.z << std::endl;
		printf("Data saved!!!!!!!!!!! \n");
		initialized = 0;
	}

	std::cin >> testNumer;


	if(testNumer == 1) {

	   initialized = 1;
	}

	if(testNumer == 2) {
	    myfile.close();
	    printf("program Closed!!!!!!!!!!! \n");
	    initialized = 0;
	    exit(0);
	}

	if(testNumer == 3) {
		printf("Break Generated !!!!!!!!!!! \n");
		myfile << 100 << " " << 0 << " " << 0 << std::endl;
	   initialized = 0;
	}
		  // Format original image
}




void ComplementaryFilterROS::motionCallback(const OdomMsg::ConstPtr& motion_msg_raw)
{
	   nav_msgs::Odometry motion =  *motion_msg_raw;
	   const ros::Time& motionTime = motion_msg_raw->header.stamp;
	   double dt;

	   dt = (ros::Time::now() - motionTime).toSec();



	   std::cout << "\n  RGBD successful Motion Time delay = " << dt <<std::endl;
	   std::cout << "\n  RGBD successful Motion Time delay = " << motion.pose.covariance[1,1] <<std::endl;

	   // Update by using the measurement returned by RGBD
       //motionWithCove.twist.covariance[7*(i-5)+j] = covariance(i,j);
	   Eigen::MatrixXd icpCovariance;
	   icpCovariance = Eigen::MatrixXd::Zero(7,7);

	   double i,j;
		for(i = 0;i<5;i++)
			for(j = 0; j <7;j++)
			{
				icpCovariance(i,j) = motion.pose.covariance[7*i+j];
			}


		for(i = 5;i<7;i++)
			for(j = 0; j <7;j++)
			{
				icpCovariance(i,j) = motion.twist.covariance[7*(i-5)+j];
			}

	  //
		Eigen::MatrixXd measurementD(7,1);
		measurementD << motion.pose.pose.position.x, \
						motion.pose.pose.position.y,\
						motion.pose.pose.position.z,\
						motion.pose.pose.orientation.w, \
						motion.pose.pose.orientation.x, \
						motion.pose.pose.orientation.y, \
						motion.pose.pose.orientation.z;\

     ////////////////////////////////////////////////////////
	 //filter_.cameraMeasureKalman(X_state_, F_, G_, P_scov_,icpCovariance, Q_ncov_, measurementD);

	//For x_state: 0 ~ 2 position imu; 3 ~ 5 velocity imu; 6 ~ 8 acceleration IMU; 9 ~ 12 quaternion_imu; 13 ~ 15 angular velocity IMU;
	  //             16 ~ 18 bias accelerator;


	//std::cout << "\n  Acceleration illustration: "<< X_state_(1) <<std::endl;
	//std::cout << "\n  Acceleration illustration: "<< X_state_(2) <<std::endl;

}

void ComplementaryFilterROS::imuCallback(const ImuMsg::ConstPtr& imu_msg)
{
  const geometry_msgs::Vector3& a = imu_msg->linear_acceleration;
  const geometry_msgs::Vector3& w = imu_msg->angular_velocity;
  const ros::Time& time = imu_msg->header.stamp;

  double da;

  da = (ros::Time::now() - time).toSec();
  // Initialize.
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    getMeasurement(a.x, -a.y, a.z, X_state_(9), X_state_(10), X_state_(11), X_state_(12));
    return;
  }

  // determine dt: either constant, or from IMU timestamp
  double dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - time_prev_).toSec();

  time_prev_ = time;



  Eigen::MatrixXd measureForEstimate(6,1);
  measureForEstimate << a.x,
				    a.y,
				    a.z,
				    w.x,
				    w.y,
				    w.z;

  filter_.statePrediction(dt, X_state_, F_, G_,measureForEstimate);

  // Pure Imu Fusion and Propogation
  //For x_state: 0 ~ 2 position imu; 3 ~ 5 velocity imu; 6 ~ 8 acceleration IMU; 9 ~ 12 quaternion_imu; 13 ~ 15 angular velocity IMU;
  //             16 ~ 18 bias accelerator;
  double ax = a.x;
  double ay = a.y;
  double az = a.z;
  double wx = w.x;
  double wy = w.y;
  double wz = w.z;

  accelerationCamera(ax, ay, az, X_state_);

  /*  std::cout << "\n  Acceleration illustration: x =  "<<  X_state_(6) <<std::endl;
  std::cout << "\n  Acceleration illustration: y = "<<  X_state_(7) <<std::endl;
  std::cout << "\n  Acceleration illustration:  z = "<<  X_state_(8) <<std::endl;
*/

  std::cout << "\n  Acceleration illustration: x =  "<<  ax <<std::endl;
  std::cout << "\n  Acceleration illustration: y = "<<   ay <<std::endl;
  std::cout << "\n  Acceleration illustration:  z = "<<  az <<std::endl;

 // X_state_(6) = ax;
//  X_state_(7) = ay - 0.3;
//  X_state_(8) = az;


  Eigen::MatrixXd measurementD(6,1);
  measurementD << ax,
				  ay,
				  az,
				  wx,
				  wy,
				  wz;



  /////////////////
  filter_.imuMeasureKalman(X_state_, F_, G_, P_scov_, Q_ncov_, measurementD);

  double distanceDelta;
  distanceDelta = sqrt((last_position_(0) - X_state_(0))*(last_position_(0) - X_state_(0)) + (last_position_(1) - X_state_(1))*(last_position_(1) - X_state_(1)) +(last_position_(2) - X_state_(2))*(last_position_(2) - X_state_(2)));

  double normVelocity;
  normVelocity = sqrt(last_Velo_(0)*last_Velo_(0) + last_Velo_(1)*last_Velo_(1) +last_Velo_(2)*last_Velo_(2) );
  if (distanceDelta>0.30)
  {
	  if (normVelocity == 0.0)
	  {
		  X_state_(0) = last_position_(0);
		  X_state_(1) = last_position_(1);
		  X_state_(2) = last_position_(2);
	  }
	  else{
		  X_state_(0) = last_position_(0) + last_Velo_(0)*0.05/normVelocity;
		  X_state_(1) = last_position_(1) + last_Velo_(1)*0.05/normVelocity;
		  X_state_(2) = last_position_(2) + last_Velo_(2)*0.05/normVelocity;
	  }

	  X_state_(3) = last_Velo_(0);
	  X_state_(4) = last_Velo_(1);
	  X_state_(5) = last_Velo_(2);
  }

  imuCount = imuCount + 1;
	//std::cout << "\n  Acceleration illustration: "<< X_state_(0) <<std::endl;
	//std::cout << "\n  Acceleration illustration: "<< X_state_(1) <<std::endl;
	//std::cout << "\n  Acceleration illustration: "<< X_state_(2) <<std::endl;
  //publish path
  publishPath(imu_msg);
}

void ComplementaryFilterROS::publishPath(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  path_msg_.header.stamp = imu_msg_raw->header.stamp;
  std::string fixed_frame_ = "/odom";
  path_msg_.header.frame_id = fixed_frame_;

  Eigen::Matrix4f temp_transformation;
  temp_transformation.setIdentity();
  // Pure Imu Fusion and Propogation
  //For x_state: 0 ~ 2 position imu; 3 ~ 5 velocity imu; 6 ~ 8 acceleration IMU; 9 ~ 12 quaternion_imu; 13 ~ 15 angular velocity IMU;
  //             16 ~ 18 bias accelerator;
  double qw, qx, qy,qz;
  qw = X_state_(9);
  qx = -X_state_(10);
  qy = -X_state_(11);
  qz = -X_state_(12);

  double unit;
  unit = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw = qw /unit;
  qx = qx /unit;
  qy = qy /unit;
  qz = qz /unit;


  temp_transformation <<  qw*qw + qx*qx - qz*qz - qy*qy, - qz*qw + qy*qx- qw*qz+ qx*qy, qy*qw + qz*qx + qx*qz + qw*qy,  X_state_(0), \
						  qx*qy + qw*qz + qz*qw + qy*qx,   qy*qy- qz*qz+ qw*qw - qx*qx, qz*qy + qy*qz - qx*qw - qw*qx, X_state_(1),\
						  qx*qz - qw*qy + qz*qx - qy*qw,   qy*qz + qz*qy+ qw*qx+ qx*qw, qz*qz - qy*qy - qx*qx + qw*qw, X_state_(2),\
						  0,                              0,                            0,                              1;
	//std::cout << "\n  Acceleration illustration: "<< temp_transformation <<std::endl;
  tf::Transform temp_path;

  tf::Matrix3x3 btm;
  btm.setValue(temp_transformation(0,0),temp_transformation(0,1),temp_transformation(0,2),
		       temp_transformation(1,0),temp_transformation(1,1),temp_transformation(1,2),
		       temp_transformation(2,0),temp_transformation(2,1),temp_transformation(2,2));

  temp_path.setOrigin(tf::Vector3(temp_transformation(0,3),temp_transformation(1,3),temp_transformation(2,3)));
  temp_path.setBasis(btm);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = imu_msg_raw->header.stamp;
  pose_stamped.header.frame_id = fixed_frame_;
  tf::poseTFToMsg(temp_path, pose_stamped.pose);

  path_msg_.poses.push_back(pose_stamped);
  path_pub_.publish(path_msg_);


  vins_pose_Pub_.publish(pose_stamped);
/*
  // publish TF
  tf::Quaternion q = tf::Quaternion(qx, qy, qz, qw);
  // Create and publish fitlered IMU message.
    boost::shared_ptr<sensor_msgs::Imu> imu_msg =
        boost::make_shared<sensor_msgs::Imu>(*imu_msg_raw);
    tf::quaternionTFToMsg(q, imu_msg->orientation);
    //For x_state: 0 ~ 2 position imu; 3 ~ 5 velocity imu; 6 ~ 8 acceleration IMU; 9 ~ 12 quaternion_imu; 13 ~ 15 angular velocity IMU;
     //             16 ~ 18 bias accelerator;
    imu_msg->angular_velocity.x = X_state_(13);
    imu_msg->angular_velocity.y = X_state_(14);
    imu_msg->angular_velocity.z = X_state_(15);

    imu_msg->linear_acceleration.x = X_state_(6);
    imu_msg->linear_acceleration.y = X_state_(7);
    imu_msg->linear_acceleration.z = X_state_(8);

    // Create and publish the ROS tf.
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
      transform.setRotation(q);

    tf_broadcaster_.sendTransform(
    tf::StampedTransform(transform,
					   imu_msg_raw->header.stamp,
					   fixed_frame_,
					   imu_msg_raw->header.frame_id)); */
}

void ComplementaryFilterROS::accelerationCamera(double& ax, double& ay, double& az, Eigen::MatrixXd x_state)
{
	// Pure Imu Fusion and Propogation
    //For x_state: 0 ~ 2 position imu; 3 ~ 5 velocity imu; 6 ~ 8 acceleration IMU; 9 ~ 12 quaternion_imu; 13 ~ 15 angular velocity IMU;
    //             16 ~ 18 bias accelerator;
	/*
	Eigen::MatrixXd gravityMatrix(4,1);
	gravityMatrix <<0,
			        0,
			        0,
			        9.8;

	Eigen::MatrixXd stateQuater(4,1);
	stateQuater << x_state(9),
			       x_state(10),
			       x_state(11),
			       x_state(12);

	Eigen::MatrixXd stateQuaterRev(4,1);
	stateQuaterRev << x_state(9),
					  - x_state(10),
					  - x_state(11),
					  - x_state(12);

	/////////////////
	Eigen::MatrixXd multResult1(4,1);
	multResult1 <<1,
			1,
			1,
			1;
	quaternionMult(gravityMatrix, stateQuaterRev, multResult1);

	///////////////
	quaternionMult(gravityMatrix, multResult1, multResult1);

	////
	ax = ax + multResult1(1);
	ay = ay + multResult1(2);
	az = az + multResult1(3);*/

	double q0 = x_state(9);
	double q1 = - x_state(10);
	double q2 = - x_state(11);
	double q3 = - x_state(12);
	double x = ax;
	double y = ay;
	double z = az;
	 ax = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z;
	 ay = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z;
	 az = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z;

	  // Initialize.
	  if (!initialized_acc_)
	  {
		  initial_ax = ax;
		  initial_ay = ay;
		  initial_az = az;
		  initialized_acc_ = true;
	    return;
	  }

	 ax = ax - initial_ax;
	 ay = ay - initial_ay;
	 az = az - initial_az;

}

void ComplementaryFilterROS::quaternionMult(Eigen::MatrixXd qauternionL, Eigen::MatrixXd& qauternionR,
		                                    Eigen::MatrixXd& multResult)
{
	Eigen::MatrixXd leftQua(3,1);
	     leftQua << qauternionL(1),
					qauternionL(2),
					qauternionL(3);

	Eigen::MatrixXd rightQua(3,1);
	     rightQua << qauternionR(1),
					 qauternionR(2),
					 qauternionR(3);

	double tempFirst, tempsec;
	tempFirst = qauternionL(0)*qauternionR(0);
	tempsec = leftQua(0)*rightQua(0) + leftQua(1)*rightQua(1) + leftQua(2)*rightQua(2);
	tempFirst = tempFirst+ - tempsec;

	Eigen::MatrixXd temp3Vect(3,1);
	temp3Vect = qauternionL(0)*rightQua + qauternionR(0)*leftQua;

	temp3Vect(0) = temp3Vect(0) + leftQua(1)*rightQua(2) - leftQua(2)*rightQua(1);
	temp3Vect(1) = temp3Vect(1) + leftQua(2)*rightQua(0) - leftQua(0)*rightQua(2);
	temp3Vect(2) = temp3Vect(2) + leftQua(0)*rightQua(1) - leftQua(1)*rightQua(0);

	////////////////////////
	multResult << tempFirst,
			      temp3Vect(0),
			      temp3Vect(1),
			      temp3Vect(2);
}

void ComplementaryFilterROS::getMeasurement(
    double ax, double ay, double az,
    double& q0_guess, double& q1_guess, double& q2_guess, double& q3_guess)
{
  // q_acc is the quaternion obtained from the acceleration vector representing
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.

  // Normalize acceleration vector.
  normalizeVector(ax, ay, az);

  if (az >=0)
  {
    q0_guess =  sqrt((az + 1) * 0.5);
    q1_guess = -ay/(2.0 * q0_guess);
    q2_guess =  ax/(2.0 * q0_guess);
    q3_guess = 0;
  }
  else
  {
    double X = sqrt((1 - az) * 0.5);
    q0_guess = -ay/(2.0 * X);
    q1_guess = X;
    q2_guess = 0;
    q3_guess = ax/(2.0 * X);
  }
}

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
  tf::Matrix3x3 btm;
  btm.setValue(trans(0,0),trans(0,1),trans(0,2),
            trans(1,0),trans(1,1),trans(1,2),
            trans(2,0),trans(2,1),trans(2,2));
  tf::Transform ret;
  ret.setOrigin(tf::Vector3(trans(0,3),trans(1,3),trans(2,3)));
  ret.setBasis(btm);
  return ret;
}


}  // namespace imu_tools
