#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
   VectorXd rmse(4);
   rmse << 0, 0, 0, 0;

   // check the validity of inputs: vector size same and size != 0
   if (estimations.size() != ground_truth.size() || estimations.size() ==0){
      std::cout << "Invalid input\n" << "estimation size: " << estimations.size();
      std::cout << "ground_truth size: " << ground_truth.size() << std::endl;
      return rmse;
  }

   for (unsigned int i=0; i < estimations.size(); ++i){
      VectorXd residual = estimations[i] - ground_truth[i];
      residual = residual.array() * residual.array();
      rmse += residual;
   }
   
   rmse = rmse / estimations.size(); // calc mean
   rmse = rmse.array().sqrt();
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);

  // Unroll state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Pre-compute some term which recur in the Jacobian
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  // Sanity check to avoid division by zero
	if (std::abs(c1) < 0.0001) {
		std::cout << "Error in Calculate Jacobian. Division by zero." << std::endl;
		return Hj;
	}

	// Actually compute Jacobian matrix
	Hj << (px / c2),				(py / c2),					0,			0,
		-(py / c1),					(px / c1),					0,			0,
		py * (vx*py - vy*px) / c3,	px * (vy*px - vx*py) / c3,	px / c2,	py / c2;

	return Hj;
}
