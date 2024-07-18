#ifndef RGC4_H
#define RGC4_H

#include "jump_controller/rgc.h"

class RGC4 : public RGC
{
public:
    RGC4(JumpRobot *Robot);

    ~RGC4();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif