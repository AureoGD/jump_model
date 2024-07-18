#ifndef JUMP_ROBOT_MODEL_H
#define JUMP_ROBOT_MODEL_H

#include <OsqpEigen/OsqpEigen.h>
#include "jump_controller/jump_robot_model.h"

class JumpRobot
{

public:
    JumpRobot(Eigen::Matrix<double, 2, 1> *_q, Eigen::Matrix<double, 2, 1> *_qd, Eigen::Matrix<double, 2, 1> *_b, Eigen::Matrix<double, 2, 1> *_db);

    ~JumpRobot();

    void RobotKinematics();

    void RobotDynamics();

    void UpdateSysMatrices();

    // void UpdateCorMtx();
    // void UpdateGMtx();

    Eigen::Matrix<double, 2, 1> *q;
    Eigen::Matrix<double, 2, 1> *dq;
    Eigen::Matrix<double, 2, 1> *b;
    Eigen::Matrix<double, 2, 1> *db;

    Eigen::Matrix<double, 2, 2> J_com, J_com1, J_com2, J_foot, M, C;
    Eigen::Matrix<double, 4, 4> HT_foot, HT_com1, HT_com2;
    Eigen::Matrix<double, 2, 1> G;
    Eigen::Matrix<double, 2, 1> com_vel;
    Eigen::Matrix<double, 2, 1> com_pos;
    Eigen::Matrix<double, 2, 1> com_pos_w;
    Eigen::Matrix<double, 2, 1> foot_pos;
    Eigen::Matrix<double, 2, 1> foot_vel;
    Eigen::Matrix<double, 2, 1> base_vel;
    Eigen::Matrix<double, 2, 1> base_pos;
    Eigen::Matrix<double, 2, 1> qU;
    Eigen::Matrix<double, 2, 1> qL;

    double m_upr, m_lwr, m_base, m_total;
};
#endif