#ifndef RGC6_H
#define RGC6_H

#include "jump_controller/rgc.h"

class RGC6 : public RGC
{
public:
    RGC6(JumpRobot *Robot);

    ~RGC6();

    void UpdateDynamicModel() override;

    void UpdateModelConstants() override;

    void DefinePhi() override;

    JumpRobot *RobotKin;

    Eigen::MatrixXd C_cons_aux, GRF_mtx, Jr, C_aux;

    Eigen::Matrix<double, 2, 1> n1;
    Eigen::Matrix<double, 2, 1> t1;
};

#endif