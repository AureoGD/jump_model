#ifndef RGC5_H
#define RGC5_H

#include "jump_controller/rgc.h"

class RGC5 : public RGC
{
public:
    RGC5(JumpRobot *Robot);

    ~RGC5();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    void DefinePhi() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx, Jr, C_aux;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif