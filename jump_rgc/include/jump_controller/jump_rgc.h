#ifndef JUMP_RGC_H
#define JUMP_RGC_H

#include <OsqpEigen/OsqpEigen.h>
#include "jump_controller/jump_robot_model.h"
#include "math.h"

class JumpRGC
{

public:
    JumpRGC(JumpRobot *JumpRobot_, Eigen::Matrix<double, 2, 1> *_q, Eigen::Matrix<double, 2, 1> *_qd, Eigen::Matrix<double, 2, 1> *_qref);

    ~JumpRGC();

    void RGCConfig(double _ts, double _Kp, double _Kd);

    bool ChooseRGCPO(int npo);

    void SetupSP(int npo);

    void SetupHSP(int npo);

    void SetupFP(int npo);

    void ClearPO();

    void ConfPO(int index);

    bool SolvePo(int npo);

    void POMatrices(int npo, const Eigen::MatrixXd &mtx_A, const Eigen::MatrixXd &mtx_B);

private:
    bool constraint = true;
    const double PI = std::atan(1.0) * 4;
    double Kp, Kd, ts, g;
    int last_po, nu, ny;

    Eigen::Matrix<double, 2, 1> *q;
    Eigen::Matrix<double, 2, 1> *qd;
    Eigen::Matrix<double, 2, 1> *qref;

    // SS matrices stance phase
    Eigen::Matrix<double, 7, 7> A_sp = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Matrix<double, 7, 2> B_sp = Eigen::Matrix<double, 7, 2>::Zero();
    Eigen::Matrix<double, 9, 9> Aa_sp = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 9, 2> Ba_sp = Eigen::Matrix<double, 9, 2>::Zero();
    Eigen::Matrix<double, 2, 9> C_sp = Eigen::Matrix<double, 2, 9>::Zero();
    Eigen::Matrix<double, 2, 9> L_sp = Eigen::Matrix<double, 2, 9>::Zero();

    // SS matrices stance phase
    Eigen::Matrix<double, 7, 7> A_hsp = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Matrix<double, 7, 2> B_hsp = Eigen::Matrix<double, 7, 2>::Zero();
    Eigen::Matrix<double, 9, 9> Aa_hsp = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 9, 2> Ba_hsp = Eigen::Matrix<double, 9, 2>::Zero();
    Eigen::Matrix<double, 2, 9> C_hsp = Eigen::Matrix<double, 2, 9>::Zero();
    Eigen::Matrix<double, 2, 9> L_hsp = Eigen::Matrix<double, 2, 9>::Zero();

    // SS matrices flight phase
    Eigen::Matrix<double, 8, 8> A_fp = Eigen::Matrix<double, 8, 8>::Zero();
    Eigen::Matrix<double, 8, 2> B_fp = Eigen::Matrix<double, 8, 2>::Zero();
    Eigen::Matrix<double, 10, 10> Aa_fp = Eigen::Matrix<double, 10, 10>::Zero();
    Eigen::Matrix<double, 10, 2> Ba_fp = Eigen::Matrix<double, 10, 2>::Zero();
    Eigen::Matrix<double, 2, 10> C_fp = Eigen::Matrix<double, 2, 10>::Zero();
    Eigen::Matrix<double, 2, 10> L_fp = Eigen::Matrix<double, 2, 10>::Zero();

    Eigen::MatrixXd Phi, G, F, H, constMatrix, aux_mtx, cons_aux, G_cons, P_cons;
    Eigen::SparseMatrix<double> hessian_sparse, linearMatrix;

    JumpRobot *_JumpRobot;

    OsqpEigen::Solver solver;
    Eigen::VectorXd x, QPSolution, delta_qref, lowerBound, upperBound;

    struct parameters_PO
    {
        int N, M, nu, ny, nx, nc;
        Eigen::MatrixXd q_ref, Qq, Uu, Vq_ref, Lcons, Ucons, V_Lcons, V_Ucons, MQq, MUu;
    };

    parameters_PO po[5];

public:
    Eigen::Matrix<double, 2, 1> refHL;
};
#endif