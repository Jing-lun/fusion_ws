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

#include "visual_imu_fusion/visual_imu_fusion.h"
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdio>
#include <cmath>
#include <iostream>

namespace VINS_CCNY {

ComplementaryFilter::ComplementaryFilter() :
    gain_acc_(0.01),
    gain_mag_(0.01),
    bias_alpha_(0.01),
    do_bias_estimation_(true),
    do_adaptive_gain_(false),
    initialized_(false),
    steady_state_(false),
    q0_(1), q1_(0), q2_(0), q3_(0),
    wx_prev_(0), wy_prev_(0), wz_prev_(0),
    wx_bias_(0), wy_bias_(0), wz_bias_(0),
    ax_bias_(0), ay_bias_(0), az_bias_(0){ }

ComplementaryFilter::~ComplementaryFilter() { }

void ComplementaryFilter::setDoBiasEstimation(bool do_bias_estimation)
{
  do_bias_estimation_ = do_bias_estimation;
}

bool ComplementaryFilter::getDoBiasEstimation() const
{
  return do_bias_estimation_;
}

void ComplementaryFilter::setDoAdaptiveGain(bool do_adaptive_gain)
{
  do_adaptive_gain_ = do_adaptive_gain;
}

bool ComplementaryFilter::getDoAdaptiveGain() const
{
  return do_adaptive_gain_;
}

bool ComplementaryFilter::setGainAcc(double gain)
{
  if (gain >= 0 && gain <= 1.0)
  {
    gain_acc_ = gain;
    return true;
  }
  else
    return false;
}
bool ComplementaryFilter::setGainMag(double gain)
{
  if (gain >= 0 && gain <= 1.0)
  {
    gain_mag_ = gain;
    return true;
  }
  else
    return false;
}

double ComplementaryFilter::getGainAcc() const 
{
  return gain_acc_;
}

double ComplementaryFilter::getGainMag() const 
{
  return gain_mag_;
}

bool ComplementaryFilter::getSteadyState() const 
{
  return steady_state_;
}

bool ComplementaryFilter::setBiasAlpha(double bias_alpha)
{
  if (bias_alpha >= 0 && bias_alpha <= 1.0)
  {
    bias_alpha_ = bias_alpha;
    return true;
  }
  else
    return false;
}

double ComplementaryFilter::getBiasAlpha() const 
{
  return bias_alpha_;
}

void ComplementaryFilter::setOrientation(
    double q0, double q1, double q2, double q3) 
{
  // Set the state to inverse (state is fixed wrt body).
  invertQuaternion(q0, q1, q2, q3, q0_, q1_, q2_, q3_);
}


double ComplementaryFilter::getAngularVelocityBiasX() const
{
  return wx_bias_;
}

double ComplementaryFilter::getAngularVelocityBiasY() const
{
  return wy_bias_;
}

double ComplementaryFilter::getAngularVelocityBiasZ() const
{
  return wz_bias_;
}



void ComplementaryFilter::statePrediction(
									double dt, Eigen::MatrixXd& x_state,
									Eigen::MatrixXd& fMatrix, Eigen::MatrixXd& gMatrix, Eigen::MatrixXd measureForEstimate)
{
	  updateBiases(measureForEstimate(0), measureForEstimate(1), measureForEstimate(2), measureForEstimate(3), measureForEstimate(4), measureForEstimate(5));
	  double wx_unb = measureForEstimate(3) - wx_bias_;
	  double wy_unb = measureForEstimate(4) - wy_bias_;
	  double wz_unb = measureForEstimate(5) - wz_bias_;

	  // Prediction.
	  double q0_pred, q1_pred, q2_pred, q3_pred;

	  q0_pred = x_state(9) + 0.5*dt*( wx_unb*x_state(10) + wy_unb*x_state(11) + wz_unb*x_state(12));
	  q1_pred = x_state(10) + 0.5*dt*(-wx_unb*x_state(9) - wy_unb*x_state(12) + wz_unb*x_state(11));
	  q2_pred = x_state(11) + 0.5*dt*( wx_unb*x_state(12) - wy_unb*x_state(9) - wz_unb*x_state(10));
	  q3_pred = x_state(12) + 0.5*dt*(-wx_unb*x_state(12) + wy_unb*x_state(10) - wz_unb*x_state(9));

	  double dq0_acc, dq1_acc, dq2_acc, dq3_acc;
	  getAccCorrection(measureForEstimate(0), measureForEstimate(1), measureForEstimate(2),
	                   q0_pred, q1_pred, q2_pred, q3_pred,
	                   dq0_acc, dq1_acc, dq2_acc, dq3_acc);

	  scaleQuaternion(gain_acc_, dq0_acc, dq1_acc, dq2_acc, dq3_acc);

// acceleration complement
//	  quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred,
//	                             dq0_acc, dq1_acc, dq2_acc, dq3_acc,
//								 x_state(9), x_state(10), x_state(11), x_state(12));

	  x_state(9) = q0_pred;
	  x_state(10) = q1_pred;
	  x_state(11) = q2_pred;
	  x_state(12) = q3_pred;


	//For x_state: 0 ~ 2 position imu; 3 ~ 5 velocity imu; 6 ~ 8 acceleration IMU; 9 ~ 12 quaternion_imu; 13 ~ 15 angular velocity IMU;
	//             16 ~ 18 bias accelerator;
	//////////////////////////////////////////////////
	Eigen::MatrixXd unit_3_;
	unit_3_ = Eigen::MatrixXd::Identity(3,3);

	Eigen::MatrixXd zero_3_;
	zero_3_ = Eigen::MatrixXd::Zero(3,3);

	Eigen::MatrixXd fMatrix_129_(9,9);
	fMatrix_129_ << unit_3_, dt*unit_3_, 0.5*dt*dt*unit_3_,\
			        zero_3_, unit_3_, dt*unit_3_,\
			        zero_3_, zero_3_, unit_3_;

	Eigen::MatrixXd x_state_129_(9,1);
	double i;
	for(i=0;i<9;i++)
	{
		x_state_129_(i) = x_state(i);
	}

	x_state_129_ = fMatrix_129_*x_state_129_;

	///////////////////////a_dot = w*a + alfa*v////////////////////////////////
	x_state_129_(6) = x_state_129_(6) + dt*(x_state(14)*x_state(8) - x_state(15)*x_state(7));
	x_state_129_(7) = x_state_129_(7) + dt*(x_state(15)*x_state(6) - x_state(13)*x_state(8));
	x_state_129_(8) = x_state_129_(8) + dt*(x_state(13)*x_state(7) - x_state(14)*x_state(13));

	////////////////////////////////////////////////////////////////////////////
	Eigen::MatrixXd qMatrix(1,4);
	qMatrix << x_state(9),
			   x_state(10),
			   x_state(11),
			   x_state(12);

	Eigen::MatrixXd wMatrix(1,3);
	wMatrix << x_state(13),
			   x_state(14),
			   x_state(15);

	Eigen::MatrixXd fQMatrix;
	fQMatrix =  Eigen::MatrixXd::Zero(4,4);

    Eigen::MatrixXd fwMatrix;
    fwMatrix =  Eigen::MatrixXd::Zero(4,3);

	Eigen::MatrixXd gQMatrix;
	gQMatrix = Eigen::MatrixXd::Identity(4,3);

    Eigen::MatrixXd gwMatrix;
	gwMatrix = Eigen::MatrixXd::Identity(3,3);

	partialDerive(fQMatrix, fwMatrix, gQMatrix, gwMatrix, qMatrix, wMatrix, dt);
	////////////////////////////////////////////////////////////////////////////
/*
	double wNorm;
	wNorm = sqrt(x_state(13)*x_state(13) + x_state(14)*x_state(14) + x_state(15)*x_state(15));

	Eigen::MatrixXd x_state_10216_(7,1);

	if (wNorm>0.008)
	{
		double sw;
		double cw;
		cw = cos(wNorm*dt/2);
		sw = sin(wNorm*dt/2);
		x_state_10216_ <<  x_state(9)*cw + (1/wNorm)*(x_state(10)*x_state(13) + x_state(11)*x_state(14) + x_state(12)*x_state(15))*sw,
						   x_state(10)*cw + (1/wNorm)*(- x_state(9)*x_state(13) - x_state(12)*x_state(14) + x_state(11)*x_state(15))*sw,
						   x_state(11)*cw + (1/wNorm)*(x_state(12)*x_state(13) - x_state(9)*x_state(14) - x_state(10)*x_state(15))*sw,
						   x_state(12)*cw + (1/wNorm)*(- x_state(10)*x_state(13) + x_state(11)*x_state(14) - x_state(9)*x_state(15))*sw,
						   x_state(13),
						   x_state(14),
						   x_state(15);

	   //x_state(9)*cw + (1/wNorm)*(x_state(10)*x_state(13) + x_state(11)*x_state(14) + x_state(12)*x_state(15))*sw,
	   //x_state(10)*cw + (1/wNorm)*(- x_state(9)*x_state(13) - x_state(12)*x_state(14) + x_state(11)*x_state(15))*sw,
	   //x_state(11)*cw + (1/wNorm)*(x_state(12)*x_state(13) - x_state(9)*x_state(14) - x_state(10)*x_state(15))*sw,
	   //x_state(12)*cw + (1/wNorm)*(- x_state(10)*x_state(13) + x_state(11)*x_state(14) - x_state(9)*x_state(15))*sw,
	}
	else
	{
		x_state_10216_ <<  x_state(9),
						   x_state(10),
						   x_state(11),
						   x_state(12),
						   x_state(13),
						   x_state(14),
						   x_state(15);
	}
*/
	///////////////////update state matrix///////////////////////
	for(i=0;i<9;i++)
	{
		x_state(i) = x_state_129_(i);
	}

	/*for(i=0;i<7;i++)
	{
		x_state(i+9) = x_state_10216_(i);
	}
*/
	///////////////////////update F matrix/////////////////////////////
	fMatrix =  Eigen::MatrixXd::Zero(19,19);
	double j;
	for (i=0;i<9;i++)
		for(j=0;j<9;j++)
		{
			fMatrix(i,j) = fMatrix_129_(i,j);
		}
	for (i=0;i<4;i++)
		for(j=0;j<4;j++)
		{
			fMatrix(i+9, j+9) = fQMatrix(i,j);
		}

	for (i=0;i<4;i++)
		for(j=0;j<3;j++)
		{
			fMatrix(i+9, j+13) = fwMatrix(i,j);
		}

	for (i=0;i<3;i++)
		for(j=0;j<3;j++)
		{
			fMatrix(i+13, j+13) = 1.0;
		}

	for (i=0;i<3;i++)
		for(j=0;j<3;j++)
		{
			fMatrix(i+16, j+16) = 1.0;
		}

	/////////
	Eigen::MatrixXd faMatrix(3,3);
	faMatrix << 0, -x_state(15), x_state(14),
			  x_state(15), 0,    x_state(13),
			  -x_state(14), x_state(13),0;
	faMatrix = faMatrix*dt;

	Eigen::MatrixXd fawMatrix(3,3);
	fawMatrix << 0, x_state(8), -x_state(7),
			    -x_state(8), 0,  x_state(6),
			    x_state(7), -x_state(6), 0;
	fawMatrix = fawMatrix*dt;

	for (i=0;i<3;i++)
		for(j=0;j<3;j++)
		{
			fMatrix(i+6, j+6) = fMatrix(i+6, j+6) + faMatrix(i,j);
		}

	for (i=0;i<3;i++)
		for(j=0;j<3;j++)
		{
			fMatrix(i+6, j+13) = fawMatrix(i,j);
		}

	///////////////////////G Matrix Update//////////////////////////////
	gMatrixUpdate(gQMatrix, gwMatrix, dt, gMatrix, x_state);
}

void ComplementaryFilter::getAccCorrection(
  double ax, double ay, double az,
  double p0, double p1, double p2, double p3,
  double& dq0, double& dq1, double& dq2, double& dq3)
{
  // Normalize acceleration vector.
  //normalizeVector(ax, ay, az);

  // Acceleration reading rotated into the world frame by the inverse predicted
  // quaternion (predicted gravity):
  double gx, gy, gz;
  rotateVectorByQuaternion(ax, ay, az,
                           p0, -p1, -p2, -p3,
                           gx, gy, gz);

  //std::cout << "\n  Gravity"<< gx <<std::endl;
 // std::cout << "\n  Gravity"<< gy <<std::endl;
 // std::cout << "\n  Gravity"<< gz <<std::endl;
  // Delta quaternion that rotates the predicted gravity into the real gravity:
  dq0 =  sqrt((gz + 1) * 0.5);
  dq1 = -gy/(2.0 * dq0);
  dq2 =  gx/(2.0 * dq0);
  dq3 =  0.0;
}

void ComplementaryFilter::partialDerive(Eigen::MatrixXd& fQMatrix, Eigen::MatrixXd& fwMatrix,
		                                Eigen::MatrixXd& gQMatrix, Eigen::MatrixXd& gwMatrix,
		                                Eigen::MatrixXd qMatrix, Eigen::MatrixXd wMatrix, double dt)
{
	///////////////////////
	double q0 = qMatrix(0);
	double q1 = qMatrix(1);
	double q2 = qMatrix(2);
	double q3 = qMatrix(3);

	double w1 = wMatrix(0);
	double w2 = wMatrix(1);
	double w3 = wMatrix(2);

	Eigen::MatrixXd unit_4_;
	unit_4_ = Eigen::MatrixXd::Identity(4,4);

	//////////////////////////
	double criterion_P;
	criterion_P = sqrt(w1*w1 + w2*w2 + w3*w3);

	double cw, sw;
	cw = cos(criterion_P*dt/2);
	sw = sin(criterion_P*dt/2);
	//std::cout << "\n  Acceleration illustration: "<<criterion_P <<std::endl;

	if (criterion_P > 0.008)
	{

		Eigen::MatrixXd omMatrix(4,4);
		omMatrix << 0, -w1, -w2, -w3,\
					 w1, 0,  -w3,  w2,\
					 w2, w3,  0 , -w1,\
					 w3, -w2, w1, 0;

		fQMatrix = cos(criterion_P*dt/2)*unit_4_ + (1/criterion_P)*sin(criterion_P*dt/2)*omMatrix;
		double temp_q;
		///////////////////
		temp_q = -q1*w1 - q2*w2 - q3*w3;

		fwMatrix(0,0) = 1/criterion_P * (-q1-w1/criterion_P*criterion_P*temp_q-q0*w1*dt/2 )*sw + temp_q*w1*dt/(2*criterion_P*criterion_P)*cw;

		fwMatrix(0,1) = 1/criterion_P * (-q2-w2/criterion_P*criterion_P*temp_q-q0*w2*dt/2 )*sw + temp_q*w2*dt/(2*criterion_P*criterion_P)*cw;

		fwMatrix(0,2) = 1/criterion_P * (-q3-w3/criterion_P*criterion_P*temp_q-q0*w3*dt/2 )*sw + temp_q*w3*dt/(2*criterion_P*criterion_P)*cw;

		///////////////
		temp_q =  q0*w1 - q2*w3 + q3*w2;
		fwMatrix(1,0) = 1/criterion_P * ( q0-w1/criterion_P*criterion_P*temp_q-q1*w1*dt/2 )*sw + temp_q*w1*dt/(2*criterion_P*criterion_P)*cw;

		fwMatrix(1,1) = 1/criterion_P * ( q3-w2/criterion_P*criterion_P*temp_q-q1*w2*dt/2 )*sw + temp_q*w2*dt/(2*criterion_P*criterion_P)*cw;

		fwMatrix(1,2) = 1/criterion_P * (-q2-w3/criterion_P*criterion_P*temp_q-q1*w3*dt/2 )*sw + temp_q*w3*dt/(2*criterion_P*criterion_P)*cw;

		//////////////
		temp_q = q0*w2 + q1*w3 - q3*w1;

		fwMatrix(2,0) = 1/criterion_P * (-q3-w1/criterion_P*criterion_P*temp_q-q2*w1*dt/2 )*sw + temp_q*w1*dt/(2*criterion_P*criterion_P)*cw;

		fwMatrix(2,1) = 1/criterion_P * ( q0-w2/criterion_P*criterion_P*temp_q-q2*w2*dt/2 )*sw + temp_q*w2*dt/(2*criterion_P*criterion_P)*cw;

		fwMatrix(2,2) = 1/criterion_P * ( q1-w3/criterion_P*criterion_P*temp_q-q2*w3*dt/2 )*sw + temp_q*w3*dt/(2*criterion_P*criterion_P)*cw;

		///////////////
		temp_q =  q0*w3 - q1*w2 + q2*w1;

		fwMatrix(3,0) = 1/criterion_P * ( q2-w1/criterion_P*criterion_P*temp_q-q3*w1*dt/2 )*sw + temp_q*w1*dt/(2*criterion_P*criterion_P)*cw;

		fwMatrix(3,1) = 1/criterion_P * (-q1-w2/criterion_P*criterion_P*temp_q-q3*w2*dt/2 )*sw + temp_q*w2*dt/(2*criterion_P*criterion_P)*cw;

		fwMatrix(3,2) = 1/criterion_P * ( q0-w3/criterion_P*criterion_P*temp_q-q3*w3*dt/2 )*sw + temp_q*w3*dt/(2*criterion_P*criterion_P)*cw;
	}

	/////////////////////////////////////////////////////////////////////
	gQMatrix << -q1, -q2, -q3, \
				 q0,  q3, -q2, \
				-q3,  q0, q1,\
				 q2, -q1, q0;
	gQMatrix = gQMatrix*dt*dt/4*sw;

	///////////////////////////////////////////////////////////////////////
	gwMatrix = dt*Eigen::MatrixXd::Identity(3,3);


}


void ComplementaryFilter::gMatrixUpdate(Eigen::MatrixXd gQMatrix, Eigen::MatrixXd gwMatrix,
		                                double dt, Eigen::MatrixXd& gMatrix, Eigen::MatrixXd x_state)
{
	//For x_state: 0 ~ 2 position imu; 3 ~ 5 velocity imu; 6 ~ 8 acceleration IMU; 9 ~ 12 quaternion_imu; 13 ~ 15 angular velocity IMU;
	//             16 ~ 18 bias accelerator;
	//////////////////////////////////////////////////
	Eigen::MatrixXd unit_3_;
	unit_3_ = Eigen::MatrixXd::Identity(3,3);

	Eigen::MatrixXd gMat_129(9,3);
	gMat_129 << unit_3_*dt*dt*dt/6.0,
			    dt*dt*0.5*unit_3_,
			    dt*unit_3_;

	Eigen::MatrixXd gtemp(3,3);
	gtemp << 0, x_state(5), -x_state(4),
			-x_state(5), 0, x_state(3),
			x_state(4), -x_state(3),0;
	gtemp = gtemp*dt;

	double i,j;
	gMatrix = Eigen::MatrixXd::Zero(19,9);
	/////////////////////////////////////////

	for (i=0;i<9;i++)
			for(j=0;j<3;j++)
			{
				gMatrix(i, j) = gMat_129(i,j);
			}

	for (i=0;i<3;i++)
			for(j=0;j<3;j++)
			{
				gMatrix(i+6, j+3) = gtemp(i,j);
			}

	for (i=0;i<4;i++)
			for(j=0;j<3;j++)
			{
				gMatrix(i+9, j+3) = gQMatrix(i,j);
			}

	for (i=0;i<3;i++)
	{
		gMatrix(i+16, i+6) = 1;
	}
}

void ComplementaryFilter::cameraMeasureKalman(Eigen::MatrixXd& x_state, Eigen::MatrixXd& fMatrix,
		                                   Eigen::MatrixXd& gMatrix, Eigen::MatrixXd& P_scov_, Eigen::MatrixXd& cameraCov_,
		                                   Eigen::MatrixXd qMatrix, Eigen::MatrixXd realMeasure)
{
	//For x_state: 0 ~ 2 position imu; 3 ~ 5 velocity imu; 6 ~ 8 acceleration IMU; 9 ~ 12 quaternion_imu; 13 ~ 15 angular velocity IMU;
    //             16 ~ 18 bias accelerator;
	/////////////////////////////////////////////

	////////////
	P_scov_ = fMatrix*P_scov_*fMatrix.transpose() + gMatrix*qMatrix*gMatrix.transpose();

	/////////////////////////////////////////////
	Eigen::MatrixXd hMatrix(7,19);
	hMatrix << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			   0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			   0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
	/////////////////Prediction of Acc and Angular////////////////////////
	Eigen::MatrixXd measurePre(7,1);
	measurePre = hMatrix*x_state;

	//////////////////////////////////////////
	cameraCov_ << 0.00001, 0, 0, 0, 0, 0, 0,
				  0, 0.00001, 0, 0, 0, 0, 0,
				  0, 0, 0.00001, 0, 0, 0, 0,
				  0, 0, 0, 0.1, 0, 0, 0,
				  0, 0, 0, 0, 0.1, 0, 0,
				  0, 0, 0, 0, 0, 0.1, 0,
				  0, 0, 0, 0, 0, 0, 0.1;
	/////////////////////////////////////
	Eigen::MatrixXd sMatrix(7,7);
	sMatrix =  hMatrix*P_scov_*hMatrix.transpose() + cameraCov_;

	///////////////////////////////////////////////
	Eigen::MatrixXd kalmanGain(19,7);
	kalmanGain = P_scov_*hMatrix.transpose()*sMatrix.inverse();

	//////////////////////////////////////////////
	Eigen::MatrixXd unit_19_;
	unit_19_ = Eigen::MatrixXd::Identity(19,19);
	x_state = x_state + kalmanGain*(realMeasure - measurePre);
	P_scov_ = (unit_19_- kalmanGain*hMatrix)*P_scov_;

	// std::cout << "\n  Kalman Gain: "<< P_scov_<<std::endl;

	 //std::cout << "\n  Notice Notice "<<std::endl;

	//////////////////////////////////////////////////////
	normalizeQuaternion(x_state(9), x_state(10), x_state(11), x_state(12));
}

void ComplementaryFilter::imuMeasureKalman(Eigen::MatrixXd& x_state, Eigen::MatrixXd& fMatrix,
		                                   Eigen::MatrixXd& gMatrix, Eigen::MatrixXd& P_scov_,
		                                   Eigen::MatrixXd qMatrix, Eigen::MatrixXd realMeasure)
{
	//For x_state: 0 ~ 2 position imu; 3 ~ 5 velocity imu; 6 ~ 8 acceleration IMU; 9 ~ 12 quaternion_imu; 13 ~ 15 angular velocity IMU;
    //             16 ~ 18 bias accelerator;
	/////////////////////////////////////////////

	//////Estimation of angular velocity bias///////////////////
	realMeasure(3) = realMeasure(3) - wx_bias_;
	realMeasure(4) = realMeasure(4) - wy_bias_;
	realMeasure(5) = realMeasure(5) - wz_bias_;

	P_scov_ = fMatrix*P_scov_*fMatrix.transpose() + gMatrix*qMatrix*gMatrix.transpose();

	////////////////////////////////////////////
	Eigen::MatrixXd rStateCov(6,6);
	rStateCov << 0.01, 0, 0, 0, 0, 0,
				 0, 0.01, 0, 0, 0, 0,
				 0, 0, 0.1, 0, 0, 0,
				 0, 0, 0, 0.01, 0, 0,
				 0, 0, 0, 0, 0.01, 0,
				 0, 0, 0, 0, 0, 0.01;

	/////////////////////////////////////////////
	Eigen::MatrixXd hMatrix(6,19);
	hMatrix << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
			   0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0,
			   0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
	/////////////////Prediction of Acc and Angular////////////////////////
	Eigen::MatrixXd measurePre(6,1);
	measurePre = hMatrix*x_state;

	/////////////////////////////////////
	Eigen::MatrixXd sMatrix(6,6);
	sMatrix =  hMatrix*P_scov_*hMatrix.transpose() + rStateCov;

	///////////////////////////////////////////////
	Eigen::MatrixXd kalmanGain(19,6);
	kalmanGain = P_scov_*hMatrix.transpose()*sMatrix.inverse();

	//////////////////////////////////////////////
	Eigen::MatrixXd unit_19_;
	unit_19_ = Eigen::MatrixXd::Identity(19,19);
	x_state = x_state + kalmanGain*(realMeasure - measurePre);
	P_scov_ = (unit_19_- kalmanGain*hMatrix)*P_scov_;

	//////////////////////////////////////////////////////
	normalizeQuaternion(x_state(9), x_state(10), x_state(11), x_state(12));

}




void ComplementaryFilter::updateBiases(double ax, double ay, double az,
                                       double wx, double wy, double wz)
{
   steady_state_ = checkState(ax, ay, az, wx, wy, wz);

  if (steady_state_)
  {
    wx_bias_ += bias_alpha_ * (wx - wx_bias_);
    wy_bias_ += bias_alpha_ * (wy - wy_bias_);
    wz_bias_ += bias_alpha_ * (wz - wz_bias_);

    ax_bias_ += 0;
    ay_bias_ += 0;
    az_bias_ += 0;
  }

  wx_prev_ = wx;
  wy_prev_ = wy;
  wz_prev_ = wz;
}

bool ComplementaryFilter::checkState(double ax, double ay, double az,
                                     double wx, double wy, double wz) const
{
  double acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
  if (fabs(acc_magnitude - kGravity) > kAccelerationThreshold)
    return false;

  if (fabs(wx - wx_prev_) > kDeltaAngularVelocityThreshold ||
      fabs(wy - wy_prev_) > kDeltaAngularVelocityThreshold ||
      fabs(wz - wz_prev_) > kDeltaAngularVelocityThreshold)
    return false;

  if (fabs(wx - wx_bias_) > kAngularVelocityThreshold ||
      fabs(wy - wy_bias_) > kAngularVelocityThreshold ||
      fabs(wz - wz_bias_) > kAngularVelocityThreshold)
    return false;

  return true;
}

void ComplementaryFilter::getMeasurement(
    double ax, double ay, double az,
    double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas)
{
  // q_acc is the quaternion obtained from the acceleration vector representing
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.

  // Normalize acceleration vector.
  normalizeVector(ax, ay, az);

  if (az >=0)
  {
    q0_meas =  sqrt((az + 1) * 0.5);
    q1_meas = -ay/(2.0 * q0_meas);
    q2_meas =  ax/(2.0 * q0_meas);
    q3_meas = 0;
  }
  else
  {
    double X = sqrt((1 - az) * 0.5);
    q0_meas = -ay/(2.0 * X);
    q1_meas = X;
    q2_meas = 0;
    q3_meas = ax/(2.0 * X);
  }
}

void normalizeVector(double& x, double& y, double& z)
{
  double norm = sqrt(x*x + y*y + z*z);

  x /= norm;
  y /= norm;
  z /= norm;
}

void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3)
{
  double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm;  
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void invertQuaternion(
  double q0, double q1, double q2, double q3,
  double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv)
{
  // Assumes quaternion is normalized.
  q0_inv = q0;
  q1_inv = -q1;
  q2_inv = -q2;
  q3_inv = -q3;
}

void scaleQuaternion(
  double gain,
  double& dq0, double& dq1, double& dq2, double& dq3)
{
	if (dq0 < 0.0)//0.9
  {
    // Slerp (Spherical linear interpolation):
    double angle = acos(dq0);
    double A = sin(angle*(1.0 - gain))/sin(angle);
    double B = sin(angle * gain)/sin(angle);
    dq0 = A + B * dq0;
    dq1 = B * dq1;
    dq2 = B * dq2;
    dq3 = B * dq3;
  }
  else
  {
    // Lerp (Linear interpolation):
    dq0 = (1.0 - gain) + gain * dq0;
    dq1 = gain * dq1;
    dq2 = gain * dq2;
    dq3 = gain * dq3;
  }

  normalizeQuaternion(dq0, dq1, dq2, dq3);  
}

void quaternionMultiplication(
  double p0, double p1, double p2, double p3,
  double q0, double q1, double q2, double q3,
  double& r0, double& r1, double& r2, double& r3)
{
  // r = p q
  r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;
  r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2;
  r2 = p0*q2 - p1*q3 + p2*q0 + p3*q1;
  r3 = p0*q3 + p1*q2 - p2*q1 + p3*q0;
}

void rotateVectorByQuaternion( 
  double x, double y, double z,
  double q0, double q1, double q2, double q3,
  double& vx, double& vy, double& vz)
{ 
  vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z;
  vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z;
  vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z;
}


}  // namespace imu_tools
