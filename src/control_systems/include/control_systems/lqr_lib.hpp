#ifndef LQR_HPP
#define LQR_HPP

#include <Eigen/Dense>
#include <iostream>

class LQR {
    public:

    //dev[x(t)] = Ax + Bu 
    //derivative_state = StateTransitionMatrix * StateVector + controlInputMa

    //Eigen Matrix Initialization

    using StateMatrix = Eigen::MatrixXd;
    using InputMatrix = Eigen::MatrixXd;
    using StateVector = Eigen::VectorXd;
    using InputVector = Eigen::VectorXd;

    //Constructor to initialize LQR with cost marrices Q,R and prediction horizon

    LQR(StateMatrix const& Q,InputMatrix const& R, int horizon);

    StateMatrix getA(double yaw,double v.double dt);
    InputMatrix getB(double yaw,double dt);
    void updateMatrices(StateMatrix const& A,InputMatrix const& B);


    //Compute the Ricati equation to update the gain of matrix x
    void computeRiccati(InputMatrix const& B,StateMatrix const& A);
    InputVector computeOptimalInput(StateVector const& state_error);
    StateMatrix K_;

    private:

    StateMatrix A_;
    InputMatrix B_;
    StateMatrix Q_;
    InputMatrix R_;
    StateMatrix P_;
    int horizon_;

}

#endif // LQR_HPP