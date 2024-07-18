#include "jump_controller/rgc2.h"

RGC2::RGC2(JumpRobot *Robot) : RobotKin(Robot)
{
    // Resize dynamic model matrices
    this->A.resize(7, 7);
    this->A.setZero();

    this->B.resize(7, 2);
    this->B.setZero();

    this->Aa.resize(9, 9);
    this->Aa.setZero();

    this->Ba.resize(9, 2);
    this->Ba.setZero();

    this->Ca.resize(2, 9);
    this->Ca.setZero();

    // q and tau
    // this->C_cons.resize(4, 9);
    // this->C_cons.setZero();

    // // q and force
    this->C_cons.resize(5, 9);
    this->C_cons.setZero();

    this->qhl.resize(2, 1);
    this->qhl.setZero();
}

RGC2::~RGC2()
{
}

void RGC2::UpdateModelConstants()
{
    /*
    x = | dr q r -g|   u = qr

    A = | K2 K1 0 0;1| B = |-K1|
        | GS 0  0  0 |     | 0 |
        | I  0  0  0 |     | 0 |
        | 0  0  0  0 |     | 0 |

    xa = | dr q r -g qra|   u = dqr

    Aa = | Ad  Bd|  Ba = | Bd|
         | 0   I |       | I |

        Ca = |0 I 0 0 0| x = |q|
*/
    this->qhl << -PI * 60 / 180, PI * 120 / 180;

    // Initialize matrices constants

    this->A.block(4, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->A(1, 6) = 1.0;

    this->Ca.block(0, 2, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    this->Aa.block(7, 7, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->Ba.block(7, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    // Initializa constraints matrix

    /*
        1ª cons: joint position
        q_min <= q <= q_max

        C_cons = |0 I 0 0 0|

    */

    this->C_cons.block(0, 2, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    /*
        2º cons: torque
        tau_min <= tau <= tau_max
        C_cons =  |GS -Kp 0 0 Kp|
    */

    // this->C_cons.block(2, 2, 2, 2) = -this->Kp * Eigen::MatrixXd::Identity(2, 2);
    // this->C_cons.block(2, 7, 2, 2) = this->Kp * Eigen::MatrixXd::Identity(2, 2);

    /*
        3ª cons: GRF
        grf_min <= grf <= grf_max

        C_cons_aux = |GS -Kp 0 0 Kp|

        C_cons =  -GRF * Jci' * C_cons_aux
    */
    this->C_cons_aux.resize(2, 9);
    this->C_cons_aux.setZero();
    this->C_cons_aux.block(0, 2, 2, 2) = -this->Kp * Eigen::MatrixXd::Identity(2, 2);
    this->C_cons_aux.block(0, 7, 2, 2) = this->Kp * Eigen::MatrixXd::Identity(2, 2);

    this->GRF_mtx.resize(3, 2);

    this->n1 << 0, 1;
    this->t1 << 1, 0;

    double a_coef = 0.9 / sqrt(2);

    this->GRF_mtx << (-a_coef * this->n1 + this->t1).transpose(),
        (a_coef * this->n1 + this->t1).transpose(),
        this->n1.transpose();

    return;
}

void RGC2::UpdateDynamicModel()
{
    // std::cout << "RGC1" << std::endl;
    // Update the dynamic matrixes

    /*
       x = | dr q r -g|  A = | K2 K1 0 0;1| B = |-K1|
                             | GS 0  0  0 |     | 0 |
                             | I  0  0  0 |     | 0 |
                             | 0  0  0  0 |     | 0 |
    */

    auto gamma_star = (RobotKin->J_com - RobotKin->J_foot).inverse();
    auto inv_J = (RobotKin->J_foot.inverse()).transpose();
    auto K1 = this->Kp * inv_J / RobotKin->m_total;
    auto K2 = this->Kd * gamma_star / RobotKin->m_total;
    this->A.block(0, 0, 2, 2) = K2;         // matlab A(1:2,1:2) = K2;
    this->A.block(0, 2, 2, 2) = K1;         // matlab A(1:2,3:4) = K1;
    this->A.block(2, 0, 2, 2) = gamma_star; // matlab A(3:4,1:2) = gammas_star;
    this->B.block(0, 0, 2, 2) = -K1;

    /*
     Using forward Euler, the dynamical model is dicretized and then aumented
      Aa = | Ad  Bd|  Ba = | Bd|
           | 0   I |       | 0 |
    */

    this->Aa.block(0, 0, 7, 7) = Eigen::MatrixXd::Identity(7, 7) + this->ts * this->A;
    this->Aa.block(0, 7, 7, 2) = this->ts * this->B;
    this->Ba.block(0, 0, 7, 2) = this->ts * this->B;

    // tau constraints
    // this->C_cons.block(2, 0, 2, 2) = -this->Kd * gamma_star;

    // GRF constraints
    this->C_cons_aux.block(0, 0, 2, 2) = -this->Kd * gamma_star;
    this->C_cons.block(2, 0, 3, 9) = -this->GRF_mtx * inv_J * this->C_cons_aux;
}