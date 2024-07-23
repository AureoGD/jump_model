#include "jump_controller/simple_rgc.h"
#include "jump_controller/rgc.h"

SimpleRGC::SimpleRGC(JumpRobot *Robot, Eigen::Matrix<double, 2, 1> *_q, Eigen::Matrix<double, 2, 1> *_qd, Eigen::Matrix<double, 2, 1> *_qr)
    : q(_q), qd(_qd), qr(_qr), _JumpRobot(Robot)
{
    // this->config = YAML::LoadFile("config.yaml");

    // if (this->config["RGC"])
    // {
    //     std::cout << "Last logged in: " << std::endl;
    // }

    this->rgc1 = new RGC1(Robot);
    this->po[0] = this->rgc1;

    this->rgc2 = new RGC2(Robot);
    this->po[1] = this->rgc2;

    this->rgc3 = new RGC3(Robot);
    this->po[2] = this->rgc3;

    this->rgc4 = new RGC4(Robot);
    this->po[3] = this->rgc4;

    this->rgc5 = new RGC5(Robot);
    this->po[4] = this->rgc5;

    this->rgc6 = new RGC6(Robot);
    this->po[5] = this->rgc6;

    this->qhl.resize(2, 1);
}

SimpleRGC::~SimpleRGC()
{
}

void SimpleRGC::RGCConfig(double _ts, double _Kp, double _Kd)
{
    // TODO - create a function that reads some loader file

    Eigen::MatrixXd Q, R, ref, Ub, Lb;

    Q.resize(2, 2);
    Q << 0.15, 0, 0, 0.15;

    R.resize(2, 2);
    R << 4.0, 0, 0, 4.0;

    Ub.resize(5, 1);
    Ub << _JumpRobot->qU, 0, OsqpEigen::INFTY, -this->g * 5 * _JumpRobot->m_total;

    Lb.resize(5, 1);
    Lb << _JumpRobot->qL, -OsqpEigen::INFTY, 0, -this->g * 0.25 * _JumpRobot->m_total;

    this->po[0]->SetInternalVariables();
    this->po[0]->SetConstants(_ts, 15, 8, _Kp, _Kd);
    this->po[0]->UpdateModelConstants();
    this->ConfPO(0);
    this->po[0]->SetWeightMatrices(Q, R);
    this->po[0]->UpdateReferences();
    this->po[0]->SetConsBounds(Lb, Ub);

    // PO 1
    this->ClearPO();

    this->po[1]->SetInternalVariables();
    this->po[1]->SetConstants(_ts, 15, 8, _Kp, _Kd);
    this->po[1]->UpdateModelConstants();
    this->ConfPO(1);
    this->po[1]->SetWeightMatrices(Q, R);
    this->po[1]->UpdateReferences();
    this->po[1]->SetConsBounds(Lb, Ub);

    // PO 2

    Ub.resize(4, 1);
    Ub << _JumpRobot->qU, 150, 150;

    Lb.resize(4, 1);
    Lb << _JumpRobot->qL, -150, -150;

    this->ClearPO();

    this->po[2]->SetInternalVariables();
    this->po[2]->SetConstants(_ts, 15, 8, _Kp, _Kd);
    this->po[2]->UpdateModelConstants();
    this->ConfPO(2);
    this->po[2]->SetWeightMatrices(Q, R);
    this->po[2]->UpdateReferences();
    this->po[2]->SetConsBounds(Lb, Ub);

    // PO 3

    this->ClearPO();

    this->po[3]->SetInternalVariables();
    this->po[3]->SetConstants(_ts, 15, 8, _Kp, _Kd);
    this->po[3]->UpdateModelConstants();
    this->ConfPO(3);
    this->po[3]->SetWeightMatrices(Q, R);
    this->po[3]->UpdateReferences();
    this->po[3]->SetConsBounds(Lb, Ub);

    // PO 4

    this->ClearPO();

    Q.resize(3, 3);
    Q << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.015;

    Ub.resize(5, 1);
    Ub << _JumpRobot->qU, 0, OsqpEigen::INFTY, -this->g * 5 * _JumpRobot->m_total;

    Lb.resize(5, 1);
    Lb << _JumpRobot->qL, -OsqpEigen::INFTY, 0, -this->g * 0.25 * _JumpRobot->m_total;

    Eigen::VectorXd nr;
    nr.resize(1, 1);
    nr.setZero();

    this->po[4]->SetInternalVariables();
    this->po[4]->SetConstants(_ts, 8, 4, _Kp, _Kd);
    this->po[4]->UpdateModelConstants();
    this->ConfPO(4);
    this->po[4]->SetWeightMatrices(Q, R);
    this->po[4]->UpdateReferences(nr);
    this->po[4]->SetConsBounds(Lb, Ub);

    // PO 5
    this->ClearPO();

    Ub.resize(4, 1);
    Ub << _JumpRobot->qU, 75, 75;

    Lb.resize(4, 1);
    Lb << _JumpRobot->qL, -75, -75;

    this->po[5]->SetInternalVariables();
    this->po[5]->SetConstants(_ts, 8, 4, _Kp, _Kd);
    this->po[5]->UpdateModelConstants();
    this->ConfPO(5);
    this->po[5]->SetWeightMatrices(Q, R);
    this->po[5]->UpdateReferences(nr);
    this->po[5]->SetConsBounds(Lb, Ub);

    this->first_conf = 1;
}

bool SimpleRGC::ChooseRGCPO(int npo)
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
    }

    if (npo == 0 || npo == 1 || npo == 4 || npo == 5)
    {
        // update the states vector |dr, q, r, g, qa|
        this->x << _JumpRobot->com_vel, *(q), _JumpRobot->com_pos, this->g, *(qr);
    }

    if (npo == 2 || npo == 3)
    {
        // update the states vector |dr, q, qa|
        this->x << *(qd), *(q), *(qr);
    }

    this->qhl = this->po[npo]->qhl;
    this->po[npo]->UpdateStates(this->x);

    this->po[npo]->UpdateOptimizationProblem(this->H, this->F, this->Ain, this->Lb, this->Ub);

    if (this->SolvePO())
    {
        *(qr) = *(qr) + this->delta_qref;
        return 1;
    }
    else
    {
        return 0;
    }
}

void SimpleRGC::ClearPO()
{
    this->solver.data()->clearLinearConstraintsMatrix();
    this->solver.data()->clearHessianMatrix();
    this->solver.clearSolver();
}

void SimpleRGC::ConfPO(int index)
{
    // first, resize the matrices

    this->x.resize(this->po[index]->nxa);
    this->x.setZero();

    this->H.resize(this->po[index]->nu * this->po[index]->M, this->po[index]->nu * this->po[index]->M);
    this->H.setZero();

    this->F.resize(1, this->po[index]->nu * this->po[index]->M);
    this->F.setZero();

    this->Ain.resize(this->po[index]->nc * this->po[index]->N, this->po[index]->nu * this->po[index]->M);
    this->Ain.setZero();

    this->Lb.resize(this->po[index]->nc * this->po[index]->N);
    this->Lb.setZero();

    this->Ub.resize(this->po[index]->nc * this->po[index]->N);
    this->Ub.setZero();

    // then, configure the solver

    this->solver.settings()->setVerbosity(0);

    this->solver.data()->setNumberOfVariables(this->po[index]->nu * this->po[index]->M);

    this->hessian_sparse = this->H.sparseView();
    this->solver.data()->clearHessianMatrix();
    this->solver.data()->setHessianMatrix(this->hessian_sparse);

    this->solver.data()->setGradient(F.transpose());

    this->solver.data()->setNumberOfConstraints(this->po[index]->nc * this->po[index]->N);
    this->linearMatrix = this->Ain.sparseView();
    this->solver.data()->setLinearConstraintsMatrix(this->linearMatrix);
    this->solver.data()->setLowerBound(this->Lb);
    this->solver.data()->setUpperBound(this->Ub);

    if (this->po[index]->nc != 0)
        this->constraints = 1;

    if (!this->first_conf)
    {
        if (!this->solver.initSolver())
            std::cout << "***************** PO " << index << " Inicialization Problem ***************** " << std::endl;
        else
            std::cout << "***************** PO " << index << " OK ***************** " << std::endl;
    }
    else
    {
        this->solver.initSolver();
    }
}

bool SimpleRGC::SolvePO()
{

    this->hessian_sparse = this->H.sparseView();

    this->solver.updateHessianMatrix(this->hessian_sparse);
    this->solver.updateGradient(this->F.transpose());

    if (this->constraints != 0)
    {
        this->linearMatrix = this->Ain.sparseView();
        this->solver.updateLinearConstraintsMatrix(this->linearMatrix);
        this->solver.updateBounds(this->Lb, this->Ub);
    }

    if (this->solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError)
    {
        if (this->solver.getStatus() != OsqpEigen::Status::Solved)
        {
            if (this->debug)
                std::cout << "Not solved" << std::endl;
            return 0;
        }

        this->QPSolution = this->solver.getSolution();
        this->delta_qref = this->QPSolution.block(0, 0, 2, 1);
        if (this->debug)
            std::cout << "Solved" << std::endl;
        return 1;
    }
    else
    {
        if (this->debug)
            std::cout << "Not solved" << std::endl;
        return 0;
    }
}