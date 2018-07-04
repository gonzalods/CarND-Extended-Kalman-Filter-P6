#include "kalman_filter.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init() {
  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);    
}

void KalmanFilter::Predict() {
  /**
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - (H_ * x_);
  MatrixXd Ht = H_.transpose();  
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
    
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double c1 = sqrt(px*px + py*py);
  VectorXd aux = x_;
  VectorXd x_pred = VectorXd(3);
  x_pred << c1,
            atan2(py, px),
            (px*vx + py*vy) / c1;

  VectorXd y = z - x_pred;
  double phi = y(1);
  while(phi < -3.1415){
      phi = phi + (2 * 3.1415); 
  }
  while(phi > 3.1415){
      phi = phi - (2 * 3.1415);
  }
  y(1) = phi;
  MatrixXd Ht = H_.transpose();  
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
    
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
  
}
