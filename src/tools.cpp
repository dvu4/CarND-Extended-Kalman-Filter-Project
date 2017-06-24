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

    // check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	
    if (estimations.size() != ground_truth.size() && ground_truth.size() == 0){
        cout << "vector size is invalid" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
    	VectorXd residual = estimations[i] - ground_truth[i];
    	residual = residual.array()*residual.array();
    	rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

	//return the result
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

	//check division by zero
	if (px == 0 && py == 0){
		Hj = MatrixXd::Zero(3,4);
	}

	//Compute the Jacobian matrix
	float den = px*px + py*py;

  while(fabs(den) < 0.0001){
    cout << "CalculateJacobian() - Error - division by zero " << endl;
    cout << "px = " << px << "and py = " << py << "... adding 0.001 and continuing" << endl;
    px += 0.001;
    py += 0.001;
    den = px*px + py*py;
  }

	Hj << px/sqrt(den), py/sqrt(den), 0, 0,
		  -py/sqrt(den), px/sqrt(den), 0, 0,
		  py*(vx*py - vy*px)/pow(den,3/2), px*(vy*px - vx*py)/pow(den,3/2), px/sqrt(den), py/sqrt(den);

	return Hj;

}
