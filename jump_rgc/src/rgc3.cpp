#include "jump_controller/rgc3.h"

RGC3::RGC3(JumpRobot *Robot) : RobotKin(Robot)
{
    // Resize dynamic model matrices
    this->A.resize(4, 4);
    this->A.setZero();

    this->B.resize(4, 2);
    this->B.setZero();

    this->Aa.resize(6, 6);
    this->Aa.setZero();

    this->Ba.resize(6, 2);
    this->Ba.setZero();

    this->Ca.resize(2, 6);
    this->Ca.setZero();

    // q and force
    this->C_cons.resize(4, 6);
    this->C_cons.setZero();

    this->qhl.resize(2, 1);
    this->qhl.setZero();
}

RGC3::~RGC3()
{
}

void RGC3::UpdateModelConstants()
{
    /*
    x = |dq q|   u = qr

    A = | Mi*(C+kd) Mi*Kp | B = |Mi*Kp|
        | I           0   |     |  0  |


    xa = |dq q qra|   u = dqr

    Aa = | Ad  Bd|  Ba = | Bd|
         | 0   I |       | I |

        Ca = |0 I 0| x = |q|
    */

    this->qhl << -PI * 30 / 180, PI * 60 / 180;

    // Initialize matrices constants

    this->A.block(2, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->Ca.block(0, 2, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    this->Aa.block(4, 4, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->Ba.block(4, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    // Initializa constraints matrix

    /*
        1ª cons: joint position
        q_min <= q <= q_max

        C_cons = |0 I 0|

    */

    this->C_cons.block(0, 2, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    /*
        2º cons: torque
        tau_min <= tau <= tau_max
        C_cons =  |-kd -Kp Kp|
    */
    this->C_cons.block(2, 0, 2, 2) = -this->Kd * Eigen::MatrixXd::Identity(2, 2);
    this->C_cons.block(2, 2, 2, 2) = -this->Kp * Eigen::MatrixXd::Identity(2, 2);
    this->C_cons.block(2, 4, 2, 2) = this->Kp * Eigen::MatrixXd::Identity(2, 2);

    return;
}

void RGC3::UpdateDynamicModel()
{
    // std::cout << "RGC1" << std::endl;
    // Update the dynamic matrixes

    /*
    x = |dq q|   u = qr

    A = |-Mi*(C+kd) -Mi*Kp | B = |Mi*Kp|
        |I             0   |     |  0  |

    */

    auto inv_m = RobotKin->M.inverse();
    this->A.block(0, 0, 2, 2) = -inv_m * (RobotKin->C + this->Kd * Eigen::MatrixXd::Identity(2, 2));
    this->A.block(0, 2, 2, 2) = -this->Kp * inv_m;
    this->B.block(0, 0, 2, 2) = this->Kp * inv_m;

    /*
     Using forward Euler, the dynamical model is dicretized and then aumented
      Aa = |Ad  Bd|  Ba = |Bd|
           |0   I |       |0 |
    */

    this->Aa.block(0, 0, 4, 4) = Eigen::MatrixXd::Identity(4, 4) + this->ts * this->A;
    this->Aa.block(0, 4, 4, 2) = this->ts * this->B;
    this->Ba.block(0, 0, 4, 2) = this->ts * this->B;
}