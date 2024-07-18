#ifndef RGC2_H
#define RGC2_H

#include "jump_controller/rgc.h"

class RGC2 : public RGC
{
public:
    RGC2(JumpRobot *Robot);

    ~RGC2();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif