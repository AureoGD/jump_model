#ifndef RGC_H
#define RGC_H

#include "jump_controller/pred_control.h"
#include "jump_controller/jump_robot_model.h"

class RGC : public PredControl
{
public:
    // RGC(JumpRobot *Robot);

    // ~RGC();

    virtual void SetConstants(double _ts, int _N, int _M, double _kp, double _kd);

    // virtual void UpdateDynamicModel();

    virtual void UpdateModelConstants() = 0;

    virtual void UpdateReferences();

    virtual void UpdateReferences(Eigen::VectorXd ref);

    Eigen::VectorXd qhl;

    double ts, Kp, Kd;
};

#endif