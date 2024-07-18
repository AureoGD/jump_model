#ifndef RGC3_H
#define RGC3_H

#include "jump_controller/rgc.h"

class RGC3 : public RGC
{
public:
    RGC3(JumpRobot *Robot);

    ~RGC3();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif