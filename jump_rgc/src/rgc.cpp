#include "jump_controller/rgc.h"

void RGC::SetConstants(double _ts, int _N, int _M, double _kp, double _kd)
{
    this->ts = _ts;
    this->Kp = _kp;
    this->Kd = _kd;

    this->N = _N;
    this->M = _M;

    this->ResizeMatrices();
}

void RGC::UpdateModelConstants()
{
    // Initialize matrices constants
}

void RGC::UpdateReferences()
{
    this->SetReference(this->qhl);
}

void RGC::UpdateReferences(Eigen::VectorXd ref)
{
    Eigen::VectorXd new_ref;
    new_ref.resize(this->qhl.rows() + ref.rows());
    new_ref << this->qhl, ref;
    this->SetReference(new_ref);
}
