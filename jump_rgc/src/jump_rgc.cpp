#include "jump_controller/jump_rgc.h"

JumpRGC::JumpRGC(JumpRobot *Robot, Eigen::Matrix<double, 2, 1> *_q, Eigen::Matrix<double, 2, 1> *_qd, Eigen::Matrix<double, 2, 1> *_qref)
    : q(_q), qd(_qd), qref(_qref), _JumpRobot(Robot)
{
    this->last_po = -1;
    this->g = -9.81;
    // Initializing constants stance phase
    this->A_sp.block(4, 5, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->A_sp(1, 6) = 1.0;
    this->C_sp.block(0, 2, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->Aa_sp.block(7, 7, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->Ba_sp.block(7, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    // Initilizing constants flight phase

    this->A_fp.block(2, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->A_fp(5, 6) = 1.0;
    this->A_fp(6, 7) = -1.0;
    this->C_fp.block(0, 2, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->Aa_fp.block(8, 8, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    this->Ba_fp.block(8, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    this->nu = 2;
    this->ny = 2;

    // ############################# PO0 ######################################
    this->po[0].ny = 2; // create a "global variable"
    this->po[0].nu = 2;
    this->po[0].nc = 3;

    this->po[0].nx = 7;
    this->po[0].N = 15;
    this->po[0].M = 8;
    this->po[0].q_ref.resize(this->po[0].ny, 1);
    this->po[0].q_ref << -PI * 30 / 180, PI * 60 / 180;
    this->po[0].Qq.resize(this->po[0].ny, this->po[0].ny);
    this->po[0].Qq << 0.15, 0.00,
        0.00, 0.15;
    this->po[0].Uu.resize(this->po[0].ny, this->po[0].ny);
    this->po[0].Uu << 4.00, 0.00,
        0.00, 4.00;

    this->po[0].Lcons.resize(3, 1);
    this->po[0].Lcons << -OsqpEigen::INFTY, 0, -this->g * 0.25 * _JumpRobot->m_total;
    this->po[0].Ucons.resize(3, 1);
    this->po[0].Ucons << 0, OsqpEigen::INFTY, -this->g * 5 * _JumpRobot->m_total;

    this->po[0].Vq_ref.resize(this->po[0].ny * this->po[0].N, 1);
    this->po[0].MQq.resize(this->po[0].ny * this->po[0].N, this->po[0].nu * this->po[0].N);
    this->po[0].MQq.setZero();
    this->po[0].MUu.resize(this->po[0].nu * this->po[0].M, this->po[0].nu * this->po[0].M);
    this->po[0].MUu.setZero();

    this->po[0].V_Lcons.resize(3 * this->po[0].N, 1); // number of variables constrained times predicte horizon
    this->po[0].V_Ucons.resize(3 * this->po[0].N, 1);

    for (int i = 0; i < this->po[0].N; i++)
    {
        this->po[0].Vq_ref.block(i * this->po[0].ny, 0, this->po[0].ny, 1) = this->po[0].q_ref;
        this->po[0].MQq.block(i * this->po[0].ny, i * this->po[0].ny, this->po[0].ny, this->po[0].ny) = this->po[0].Qq;
        if (i < this->po[0].M)
        {
            this->po[0].MUu.block(i * this->po[0].nu, i * this->po[0].nu, this->po[0].nu, this->po[0].nu) = this->po[0].Uu;
        }
        this->po[0].V_Lcons.block(i * this->po[0].nc, 0, 3, 1) = this->po[0].Lcons;
        this->po[0].V_Ucons.block(i * this->po[0].nc, 0, 3, 1) = this->po[0].Ucons;
    }

    // std::cout << this->po[0].V_Lcons << std::endl;
    // ############################# PO1 ######################################
    this->po[1].ny = 2; // create a "global variable"
    this->po[1].nu = 2;
    this->po[1].nc = 3; // number of constraints

    this->po[1].nx = 7;
    this->po[1].N = 15;
    this->po[1].M = 8;
    this->po[1].q_ref.resize(this->po[1].ny, 1);
    this->po[1].q_ref << -PI * 60 / 180, PI * 120 / 180;
    this->po[1].Qq.resize(this->po[1].ny, this->po[1].ny);
    this->po[1].Qq << 0.15, 0.00,
        0.00, 0.15;
    this->po[1].Uu.resize(this->po[1].ny, this->po[1].ny);
    this->po[1].Uu << 4.00, 0.00,
        0.00, 4.00;

    this->po[1].Lcons.resize(3, 1);
    this->po[1].Lcons << -OsqpEigen::INFTY, 0, -this->g * 0.25 * _JumpRobot->m_total;
    this->po[1].Ucons.resize(3, 1);
    this->po[1].Ucons << 0, OsqpEigen::INFTY, -this->g * 5 * _JumpRobot->m_total;

    this->po[1].Vq_ref.resize(this->po[1].ny * this->po[1].N, 1);
    this->po[1].MQq.resize(this->po[1].ny * this->po[0].N, this->po[1].nu * this->po[1].N);
    this->po[1].MQq.setZero();
    this->po[1].MUu.resize(this->po[1].nu * this->po[0].M, this->po[1].nu * this->po[1].M);
    this->po[1].MUu.setZero();

    this->po[1].V_Lcons.resize(3 * this->po[1].N, 1); // number of variables constrained times predicte horizon
    this->po[1].V_Ucons.resize(3 * this->po[1].N, 1);

    for (int i = 0; i < this->po[1].N; i++)
    {
        this->po[1].Vq_ref.block(i * this->po[1].ny, 0, this->po[1].ny, 1) = this->po[1].q_ref;
        this->po[1].MQq.block(i * this->po[1].ny, i * this->po[1].ny, this->po[1].ny, this->po[1].ny) = this->po[1].Qq;
        if (i < this->po[1].M)
        {
            this->po[1].MUu.block(i * this->po[1].nu, i * this->po[1].nu, this->po[1].nu, this->po[1].nu) = this->po[1].Uu;
        }
        this->po[1].V_Lcons.block(i * this->po[1].nc, 0, 3, 1) = this->po[1].Lcons;
        this->po[1].V_Ucons.block(i * this->po[1].nc, 0, 3, 1) = this->po[1].Ucons;
    }

    // ############################# PO2 ######################################
    this->po[2].ny = 2; // create a "global variable"
    this->po[2].nu = 2;
    this->po[2].nc = 2; // number of constraints

    this->po[2].nx = 8;
    this->po[2].N = 15;
    this->po[2].M = 8;
    this->po[2].q_ref.resize(this->po[2].ny, 1);
    this->po[2].q_ref << -PI * 30 / 180, PI * 60 / 180;
    this->po[2].Qq.resize(this->po[2].ny, this->po[2].ny);
    this->po[2].Qq << 0.15, 0.00,
        0.00, 0.15;
    this->po[2].Uu.resize(this->po[2].ny, this->po[2].ny);
    this->po[2].Uu << 4.00, 0.00,
        0.00, 4.00;

    this->po[2].Lcons.resize(2, 1);
    this->po[2].Lcons << -100, -100;
    this->po[2].Ucons.resize(2, 1);
    this->po[2].Ucons << 100, 100;

    this->po[2].Vq_ref.resize(this->po[2].ny * this->po[2].N, 1);
    this->po[2].MQq.resize(this->po[2].ny * this->po[0].N, this->po[2].nu * this->po[2].N);
    this->po[2].MQq.setZero();
    this->po[2].MUu.resize(this->po[2].nu * this->po[0].M, this->po[2].nu * this->po[2].M);
    this->po[2].MUu.setZero();
    this->po[2].V_Lcons.resize(2 * this->po[2].N, 1); // number of variables constrained times predicte horizon
    this->po[2].V_Ucons.resize(2 * this->po[2].N, 1);

    for (int i = 0; i < this->po[2].N; i++)
    {
        this->po[2].Vq_ref.block(i * this->po[2].ny, 0, this->po[2].ny, 1) = this->po[2].q_ref;
        this->po[2].MQq.block(i * this->po[2].ny, i * this->po[2].ny, this->po[2].ny, this->po[2].ny) = this->po[2].Qq;
        if (i < this->po[2].M)
        {
            this->po[2].MUu.block(i * this->po[2].nu, i * this->po[2].nu, this->po[2].nu, this->po[2].nu) = this->po[2].Uu;
        }
        this->po[2].V_Lcons.block(i * this->po[2].nc, 0, 2, 1) = this->po[2].Lcons;
        this->po[2].V_Ucons.block(i * this->po[2].nc, 0, 2, 1) = this->po[2].Ucons;
    }

    // ############################# PO3 ######################################
    this->po[3].ny = 2; // create a "global variable"
    this->po[3].nu = 2;
    this->po[3].nc = 2; // number of constraints

    this->po[3].nx = 8;
    this->po[3].N = 15;
    this->po[3].M = 8;
    this->po[3].q_ref.resize(this->po[3].ny, 1);
    this->po[3].q_ref << -PI * 60 / 180, PI * 120 / 180;
    this->po[3].Qq.resize(this->po[3].ny, this->po[3].ny);
    this->po[3].Qq << 0.15, 0.00,
        0.00, 0.15;
    this->po[3].Uu.resize(this->po[3].ny, this->po[3].ny);
    this->po[3].Uu << 4.00, 0.00,
        0.00, 4.00;

    this->po[3].Lcons.resize(2, 1);
    this->po[3].Lcons << -100, -100;
    this->po[3].Ucons.resize(2, 1);
    this->po[3].Ucons << 100, 100;

    this->po[3].Vq_ref.resize(this->po[3].ny * this->po[3].N, 1);
    this->po[3].MQq.resize(this->po[3].ny * this->po[0].N, this->po[3].nu * this->po[3].N);
    this->po[3].MQq.setZero();
    this->po[3].MUu.resize(this->po[3].nu * this->po[0].M, this->po[3].nu * this->po[3].M);
    this->po[3].MUu.setZero();
    this->po[3].V_Lcons.resize(2 * this->po[3].N, 1); // number of variables constrained times predicte horizon
    this->po[3].V_Ucons.resize(2 * this->po[3].N, 1);

    for (int i = 0; i < this->po[3].N; i++)
    {
        this->po[3].Vq_ref.block(i * this->po[3].ny, 0, this->po[3].ny, 1) = this->po[3].q_ref;
        this->po[3].MQq.block(i * this->po[3].ny, i * this->po[3].ny, this->po[3].ny, this->po[3].ny) = this->po[3].Qq;
        if (i < this->po[3].M)
        {
            this->po[3].MUu.block(i * this->po[3].nu, i * this->po[3].nu, this->po[3].nu, this->po[3].nu) = this->po[3].Uu;
        }
        this->po[3].V_Lcons.block(i * this->po[3].nc, 0, 2, 1) = this->po[3].Lcons;
        this->po[3].V_Ucons.block(i * this->po[3].nc, 0, 2, 1) = this->po[3].Ucons;
    }
}

JumpRGC::~JumpRGC()
{
}

void JumpRGC::RGCConfig(double _ts, double _Kp, double _Kd)
{
    this->ts = _ts;
    // std::cout << "MPC period: "<< this->ts<<std::endl;
    this->Kp = _Kp;
    this->Kd = _Kd;

    // Stance phase L matrix

    this->L_sp.block(0, 2, 2, 2) = -_Kp * Eigen::MatrixXd::Identity(2, 2);
    this->L_sp.block(0, 7, 2, 2) = _Kp * Eigen::MatrixXd::Identity(2, 2);

    // Flight phase L matrix
    this->L_fp.block(0, 0, 2, 2) = -_Kd * Eigen::MatrixXd::Identity(2, 2);
    this->L_fp.block(0, 2, 2, 2) = -_Kp * Eigen::MatrixXd::Identity(2, 2);
    this->L_fp.block(0, 8, 2, 2) = _Kp * Eigen::MatrixXd::Identity(2, 2);
}

bool JumpRGC::ChooseRGCPO(int npo)
{
    // verify if the PO is the same that the used before
    if (npo != this->last_po)
    {
        if (this->solver.isInitialized())
        {
            this->ClearPO();
        }
        this->ConfPO(npo);
        this->last_po = npo;
        this->refHL = this->po[npo].q_ref;
    }
    if (npo == 0)
    {
        this->SetupSP(npo);
    }
    else if (npo == 1)
    {
        this->SetupSP(npo);
    }
    else if (npo == 2)
    {
        this->SetupFP(npo);
    }
    else if (npo == 3)
    {
        this->SetupFP(npo);
    }

    if (!this->SolvePo())
    {
        return 0;
    }
    else
    {
        *(qref) = *(qref) + this->delta_qref;
        return 1;
    }
}

void JumpRGC::SetupSP(int npo)
{

    auto gamma_star = (_JumpRobot->J_com - _JumpRobot->J_foot).inverse();
    auto inv_J = (_JumpRobot->J_foot.inverse()).transpose();
    auto K1 = this->Kp * inv_J / _JumpRobot->m_total;
    auto K2 = this->Kd * gamma_star / _JumpRobot->m_total;
    this->A_sp.block(0, 0, 2, 2) = K2;         // matlab A(1:2,1:2) = K2;
    this->A_sp.block(0, 2, 2, 2) = K1;         // matlab A(1:2,3:4) = K1;
    this->A_sp.block(2, 0, 2, 2) = gamma_star; // matlab A(3:4,1:2);
    this->B_sp.block(0, 0, 2, 2) = -K1;

    this->Aa_sp.block(0, 0, 7, 7) = Eigen::MatrixXd::Identity(7, 7) + this->ts * this->A_sp;
    this->Ba_sp.block(0, 0, 7, 2) = this->ts * this->B_sp;

    Eigen::Matrix<double, 3, 2> Cf;
    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
    // Eigen::Matrix<double, 3, 2> Cf;
    n1 << 0, 1;
    t1 << 1, 0;
    double a_coef = 0.9 / sqrt(2);

    Cf << (-a_coef * n1 + t1).transpose(), (a_coef * n1 + t1).transpose(), n1.transpose();

    auto FC_max = -Cf * inv_J;

    this->L_sp.block(0, 0, 2, 2) = -this->Kd * gamma_star;

    this->Phi.block(0, 0, 2, 9) = this->C_sp * this->Aa_sp;

    this->P_cons.block(0, 0, 2, 9) = this->L_sp;

    for (int i = 1; i < this->po[npo].N; i++) // check i<N ?
    {
        this->Phi.block(2 * i, 0, 2, 9) = this->Phi.block(2 * (i - 1), 0, 2, 9) * this->Aa_sp;
        this->P_cons.block(2 * i, 0, 2, 9) = this->P_cons.block(2 * (i - 1), 0, 2, 9) * this->Aa_sp;
    }

    for (int i = 0; i < this->po[npo].N; i++)
    {
        if (i == 0)
        {
            // std::cout << i << std::endl;
            this->aux_mtx = this->C_sp * this->Ba_sp;
            this->cons_aux = FC_max * this->Kp * Eigen::MatrixXd::Identity(2, 2);
        }
        else
        {

            this->aux_mtx = this->Phi.block(2 * (i - 1), 0, 2, 9) * this->Ba_sp;
            this->cons_aux = FC_max * this->P_cons.block(2 * (i - 1), 0, 2, 9) * this->Ba_sp;
        }

        for (int j = 0; j < this->po[npo].M; j++)
        {
            if (2 * (i + j) <= this->po[npo].M * 2)
            {
                this->G.block((i + j) * 2, j * 2, 2, 2) = this->aux_mtx;
                this->G_cons.block((i + j) * 3, j * 2, 3, 2) = this->cons_aux;
            }
        }
    }
    // std::cout << "heu" << std::endl;
    // update the states vector [dr, q, r, g, qa]
    this->x << _JumpRobot->com_vel,
        *(q),
        _JumpRobot->com_pos,
        this->g,
        *(qref);

    //  update the MPC matrix
    this->H = 2 * (this->G.transpose() * this->po[npo].MQq * this->G + this->po[npo].MUu);

    this->F = 2 * (((this->Phi * this->x) - this->po[npo].Vq_ref).transpose()) * this->po[npo].MQq * this->G;
}

void JumpRGC::SetupFP(int npo)
{
    auto inv_m = _JumpRobot->M.inverse();
    this->A_fp.block(0, 0, 2, 2) = -inv_m * (_JumpRobot->C + this->Kd * Eigen::MatrixXd::Identity(2, 2));
    this->A_fp.block(0, 2, 2, 2) = -this->Kp * inv_m;
    this->A_fp.block(4, 0, 2, 2) = _JumpRobot->J_com;

    this->B_fp.block(0, 0, 2, 2) = this->Kp * inv_m;

    this->Aa_fp.block(0, 0, 8, 8) = Eigen::MatrixXd::Identity(8, 8) + this->ts * this->A_fp;

    this->Ba_fp.block(0, 0, 8, 2) = this->ts * this->B_fp;

    this->Phi.block(0, 0, 2, 10) = this->C_fp * this->Aa_fp;
    this->P_cons.block(0, 0, 2, 10) = this->L_fp;

    for (int i = 1; i < this->po[npo].N; i++) // check i<N ?
    {
        this->Phi.block(2 * i, 0, 2, 10) = this->Phi.block(2 * (i - 1), 0, 2, 10) * this->Aa_fp;
        this->P_cons.block(2 * i, 0, 2, 10) = this->P_cons.block(2 * (i - 1), 0, 2, 10) * this->Aa_fp;
    }

    for (int i = 0; i < this->po[npo].N; i++)
    {
        if (i == 0)
        {
            this->aux_mtx = this->C_fp * this->Ba_fp;
            this->cons_aux = this->Kp * Eigen::MatrixXd::Identity(2, 2);
        }
        else
        {
            this->aux_mtx = this->Phi.block(2 * (i - 1), 0, 2, 10) * this->Ba_fp;
            this->cons_aux = this->P_cons.block(2 * (i - 1), 0, 2, 10) * this->Ba_fp;
        }

        for (int j = 0; j < this->po[npo].M; j++)
        {
            if (2 * (i + j) <= this->po[npo].M * 2)
            {
                this->G.block((i + j) * 2, j * 2, 2, 2) = this->aux_mtx;
                this->G_cons.block((i + j) * 2, j * 2, 2, 2) = this->cons_aux;
            }
        }
    }

    // std::cout << this->G_cons << std::endl;

    // update the states vector [dq; q; r_Com_W; dz; g; qa]

    this->x << *(qd),
        *(q),
        _JumpRobot->com_pos_w,
        (*_JumpRobot->db)(1),
        this->g,
        *(qref);

    //  update the MPC matrix
    this->H = 2 * (this->G.transpose() * this->po[npo].MQq * this->G + this->po[npo].MUu);

    this->F = 2 * (((this->Phi * this->x) - this->po[npo].Vq_ref).transpose()) * this->po[npo].MQq * this->G;

    if (this->constraint)
    {
        this->lowerBound = this->po[npo].V_Lcons - this->P_cons * this->x;
        this->upperBound = this->po[npo].V_Ucons - this->P_cons * this->x;
    }
    // std::cout << this->G_cons << std::endl;
}

void JumpRGC::ClearPO()
{

    this->solver.data()->clearLinearConstraintsMatrix();
    this->solver.data()->clearHessianMatrix();
    this->solver.clearSolver();
}

void JumpRGC::ConfPO(int index)
{

    // resize state vector
    this->x.resize(this->po[index].nx + this->po[index].ny, 1);
    this->x.setZero();

    // resize dynamic matrix
    this->G.resize(this->po[index].ny * this->po[index].N, this->po[index].nu * this->po[index].M);
    this->G.setZero();

    this->G_cons.resize(this->po[index].nc * this->po[index].N, this->po[index].nu * this->po[index].M);
    this->G_cons.setZero();

    this->lowerBound.resize(this->po[index].nc * this->po[index].N, 1);
    this->lowerBound.setZero();

    this->upperBound.resize(this->po[index].nc * this->po[index].N, 1);
    this->upperBound.setZero();

    // resize free response matrix
    this->Phi.resize(this->po[index].ny * this->po[index].N, this->po[index].nx + this->po[index].ny);
    this->Phi.setZero();

    this->P_cons.resize(this->po[index].ny * this->po[index].N, this->po[index].nx + this->po[index].ny);
    this->P_cons.setZero();

    this->H.resize(this->po[index].nu * this->po[index].M, this->po[index].nu * this->po[index].M);
    this->H.setZero();

    this->F.resize(1, this->po[index].nu * this->po[index].M);
    this->F.setZero();

    this->aux_mtx.resize(this->po[index].ny, this->po[index].nx + this->po[index].ny);
    this->cons_aux.resize(this->po[index].ny, this->po[index].nx + this->po[index].ny);

    this->solver.settings()->setVerbosity(false);

    // number of output times the control horizon
    solver.data()->setNumberOfVariables(this->po[index].nu * this->po[index].M);

    if (this->constraint)
    {
        solver.data()->setNumberOfConstraints(this->po[index].nc * this->po[index].N);
        this->linearMatrix = this->G_cons.sparseView();
        solver.data()->setLinearConstraintsMatrix(this->linearMatrix);
        solver.data()->setLowerBound(this->lowerBound);
        solver.data()->setUpperBound(this->upperBound);
    }
    else
    {
        solver.data()->setNumberOfConstraints(0);
    }

    this->hessian_sparse = this->H.sparseView();

    solver.data()->clearHessianMatrix();
    solver.data()->setHessianMatrix(this->hessian_sparse);
    solver.data()->setGradient(F.transpose());

    if (!this->solver.initSolver())
        std::cout << "***************** PO Inicialization Problem ***************** " << std::endl;
}

bool JumpRGC::SolvePo()
{
    this->hessian_sparse = this->H.sparseView();
    this->solver.updateHessianMatrix(this->hessian_sparse);
    this->solver.updateGradient(this->F.transpose());

    if (this->constraint)
    {
        this->linearMatrix = this->G_cons.sparseView();
        this->solver.updateLinearConstraintsMatrix(this->linearMatrix);
        solver.updateBounds(this->lowerBound, this->upperBound);
    }

    if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError)
    {
        this->QPSolution = this->solver.getSolution();
        this->delta_qref = this->QPSolution.block(0, 0, this->nu, 1);
        // std::cout << "Solved" << std::endl;
        return 1;
    }
    else
    {
        // std::cout << "Not solved" << std::endl;
        return 0;
    }
}
