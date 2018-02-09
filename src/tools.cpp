#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  if (estimations.size() == 0) {
    cout << "the estimation vector size should not be zero";
    return rmse;
  }
  
  if (estimations.size() != ground_truth.size())
    return rmse;
  //accumulate squared residuals
  VectorXd residul(4);
  for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    residul = estimations[i] - ground_truth[i];
    residul = residul.array() * residul.array();
    rmse += residul;
  }
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //TODO: YOUR CODE HERE
    float div = px * px + py * py;
    float sqt_div = sqrt(div);
    float vp = vx * py - vy * px;
    float div2 = div * sqt_div;//pow(div, 1.5);
    float a01 = px / sqt_div;
    float a02 = py / sqt_div;
    float a10 = (-1.0) * py / div;
    float a11 = px / div;
    float a20 = py * vp / div2;
    float a21 = px * (vy * px - vx * py) / div2;
    float a22 = px / sqt_div;
    float a23 = py / sqt_div;
    //check division by zero
    if (px == 0 && py == 0)
    {
        Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
    }
    else
    {
        Hj << a01, a02, 0, 0,
        a10, a11, 0, 0,
        a20, a21, a22, a23;
    }
    
    //compute the Jacobian matrix
    
    return Hj;
}
