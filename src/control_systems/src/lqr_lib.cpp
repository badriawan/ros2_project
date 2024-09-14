#include "lqr_lib.hpp"

LQR::LQR(StateMatrix const& Q,InputMatrix const& R, int horizon):Q_(Q),R_(R),horizon_(horizon){
//constructor to define the variables
    A_ = StateMatrix::Zero(3,3);
    B_ = InputMatrix::Zero(3,2);
    Q_ = StateMatrix::Zero(3,3);
    R_ = InputMatrix::Zero(3,2);
    P_ = Q_;
    K_ = StateMatrix::Zero(B_.cols(),A_.rows());

}

LQR::StateMatrix LQR::getA(double yaw,double v,double dt){
//logic to define the transition dynamical matrix A

    StateMatrix A(3, 3);
    A << 1, 0, -v * sin(yaw) * dt,
         0, 1, v * cos(yaw) * dt,
         0, 0, 1;
    return A;
}



//logic to define the controller input matrix B
LQR::InputMatrix LQR::getB(double yaw, double dt) {

    InputMatrix B(3, 2);
    B << cos(yaw) * dt, 0,
         dt * sin(yaw), 0,
         0, dt;
    return B;

}



void LQR::updateMatrices(StateMatrix const& A,InputMatrix const& B){
//logic to updating the matrix


    A_ = A;
    B_ = B;

}

void LQR::computeRiccati(InputMatrix const& B,StateMatrix const& A){
//compute the ricatti and get P and K

    P_ = Q_;
    B_ = B;
    A_ = A;

    for (int i = horizon_; i > 0; --i) {
        Eigen::MatrixXd Y = R_ + B_.transpose() * P_ * B_;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
        P_ = Q_ + A_.transpose() * P_ * A_ - (A_.transpose() * P_ * B_) * Yinv * (B_.transpose() * P_ * A_);
    }

    Eigen::MatrixXd Y = R_ + B_.transpose() * P_ * B_;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
    K_ = Yinv * B_.transpose() * P_ * A_;

}



LQR::InputVector LQR::computeOptimalInput(StateVector const& state_error){
//compute u as an input then convert it to vector . The input are velocity linear and angular

    InputVector u = -K_ * state_error;
    return u;

}





