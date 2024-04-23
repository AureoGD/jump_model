#include "jump_controller/jump_low_level_controller.h"
#include "gz/sim/Util.hh"
#include "gz/sim/Model.hh"

LowLevelController::LowLevelController()
{
    gzmsg << "#### Low Level Controller #### " << std::endl;
}

LowLevelController::~LowLevelController()
{
}

void LowLevelController::Configure(double _Kp, double _Kd)
{
    // como passar para a função configure o vetor das referencias e das posições (ponteiro).
    // Desta foma, na função update é passado apenas o indice da junta que deve ser calculado o torque,
    // pois facilita para calcular o valor de tau G
    gzmsg << "#### Configuring Low Level Controller #### " << std::endl;
    this->Kp = _Kp;
    this->Kd = _Kd;
}

double LowLevelController::ComputeControl(double _q, double _qr, double _dq, double _dqr)
{
    double val = this->Kp * (_qr - _q) - this->Kd * _dq;
    // double val =  this->Kp*(_qr-_q);

    if (val > 500)
    {
        return 500;
    }
    else if (val < -500)
    {
        return -500;
    }
    else
    {
        return val;
    }
}
