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
	rmse << 0, 0, 0, 0;

	// Input Sanity Check
	if(estimations.size() == 0 || (estimations.size() != ground_truth.size())){
		std::cout << "Invalid estimation or ground_truth inputs";
		return rmse;
	} 

	// accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
		VectorXd residuals = estimations[i] - ground_truth[i];

		residuals = residuals.array() * residuals.array();
		rmse += residuals;
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
	// negative sanity check result
	MatrixXd Hj(3,4);
	Hj << 0, 0, 0, 0,
		  0, 0, 0, 0,
		  0, 0, 0, 0;

	float px = x_state[0];
	float py = x_state[1];
	float vx = x_state[2];
	float vy = x_state[3];

	if (px == 0 && py == 0){
		return Hj;
	}

	float px2py2 = px * px + py * py;
	Hj(0, 0) = px / sqrt(px2py2);
	Hj(0, 1) = py / sqrt(px2py2);

	Hj(1, 0) = -py / px2py2;
	Hj(1, 1) = px / px2py2;

	Hj(2, 0) = py * (vx*py - vy*px) / pow(px2py2, 3/2.0);
	Hj(2, 1) = px * (vy*px - vx*py) / pow(px2py2, 3/2.0);

	Hj(2, 2) = px / sqrt(px2py2);
	Hj(2, 3) = py / sqrt(px2py2);

	return Hj;
}
