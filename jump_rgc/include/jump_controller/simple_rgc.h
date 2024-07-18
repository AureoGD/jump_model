#ifndef SIMPLE_RGC_H
#define SIMPLE_RGC_H

#include "jump_controller/pred_control.h"
#include "jump_controller/rgc.h"
#include "jump_controller/rgc1.h"
#include "jump_controller/rgc2.h"
#include "jump_controller/rgc3.h"
#include "jump_controller/rgc4.h"
#include "jump_controller/rgc5.h"
#include "jump_controller/rgc6.h"

#include "jump_controller/jump_robot_model.h"

// #include "yaml-cpp/yaml.h"

#include <OsqpEigen/OsqpEigen.h>
#include "math.h"

class SimpleRGC
{
public:
    SimpleRGC(JumpRobot *JumpRobot_, Eigen::Matrix<double, 2, 1> *_q, Eigen::Matrix<double, 2, 1> *_qd, Eigen::Matrix<double, 2, 1> *_qr);

    ~SimpleRGC();

    void RGCConfig(double _ts, double _Kp, double _Kd);

    bool ChooseRGCPO(int npo);

    bool SolvePO();

    Eigen::VectorXd qhl;

    Eigen::Matrix<double, 2, 1> *q, *qd, *qr;
    Eigen::Matrix<double, 2, 1> delta_qref;
    Eigen::VectorXd QPSolution;

private:
    // YAML::Node config;

    void ConfPO(int index);

    void ClearPO();

    RGC1 *rgc1;
    RGC2 *rgc2;
    RGC3 *rgc3;
    RGC4 *rgc4;
    RGC5 *rgc5;
    RGC6 *rgc6;

    JumpRobot *_JumpRobot;

    RGC *po[6];

    // Eigen::VectorXd qhl;

    int last_po = -1;

    const double g = -9.81;

    bool constraints = 0;

    bool first_conf = 0;

    Eigen::MatrixXd H, F, Ain;

    Eigen::VectorXd x, Ub, Lb;

    Eigen::SparseMatrix<double> hessian_sparse, linearMatrix;

    OsqpEigen::Solver solver;

    bool debug = 0;
};

#endif